#include "TrumaiNetBoxApp.h"
#include "TrumaStatusFrameBuilder.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "helpers.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.TrumaiNetBoxApp";

static constexpr uint32_t CLOCK_SYNC_DELAY_US = 30 * 1000 * 1000;
static constexpr uint32_t INIT_RETRY_DELAY_US = 5 * 1000 * 1000;
static constexpr uint32_t UPDATE_RETRY_DELAY_US = 5 * 1000 * 1000;

TrumaiNetBoxApp::TrumaiNetBoxApp() {
  this->airconAuto_.set_parent(this);
  this->airconManual_.set_parent(this);
  this->clock_.set_parent(this);
  // this->config_.set_parent(this);
  this->heater_.set_parent(this);
  this->timer_.set_parent(this);
}

void TrumaiNetBoxApp::update() {
  this->airconAuto_.update();
  this->airconManual_.update();
  this->clock_.update();
  this->config_.update();
  this->heater_.update();
  this->timer_.update();

  LinBusProtocol::update();

#ifdef USE_TIME
  auto init_received_snapshot =
      this->init_received_.load(std::memory_order_relaxed);

  if (this->time_ != nullptr &&
      !this->update_status_clock_done &&
      init_received_snapshot > 0) {

    if ((micros() - init_received_snapshot) >
        CLOCK_SYNC_DELAY_US) {

      this->update_status_clock_done = true;
      this->clock_.action_write_time();
    }
  }
#endif
}


/*
 * ============================================================
 * IDENTITE LIN INET BOX
 * ============================================================
 *
 * IMPORTANT D4 2021 :
 *
 * On conserve l'identité réelle d'une Truma iNet Box :
 *
 *   17 46 00 1F
 *
 * On NE prend PAS l'identité 17 46 10 03 observée sur le bus,
 * car un autre équipement Truma répond déjà à cette identité.
 *
 * ============================================================
 */
const std::array<uint8_t, 4>
TrumaiNetBoxApp::lin_identifier() {

  return {
      0x17,  // Supplier ID
      0x46,  // Supplier ID
      0x00,  // Function ID
      0x1F   // iNet Box
  };
}


void TrumaiNetBoxApp::lin_heartbeat() {
  this->device_registered_.store(
      micros(),
      std::memory_order_relaxed
  );
}


void TrumaiNetBoxApp::lin_reset_device() {
  LinBusProtocol::lin_reset_device();

  this->device_registered_.store(
      micros(),
      std::memory_order_relaxed
  );

  this->init_received_.store(
      0,
      std::memory_order_relaxed
  );

  this->airconAuto_.reset();
  this->airconManual_.reset();
  this->clock_.reset();
  this->config_.reset();
  this->heater_.reset();
  this->timer_.reset();

  this->update_time_.store(
      0,
      std::memory_order_relaxed
  );
}


bool TrumaiNetBoxApp::answer_lin_order_(const uint8_t pid) {

  /*
   * Alive / notification iNet Box.
   *
   * Si aucune réponse diagnostic n'attend dans la queue et
   * aucune mise à jour applicative n'est en attente :
   *
   *   FE FF FF FF FF FF FF FF
   *
   * Sinon on laisse le premier octet à FF pour signaler
   * quelque chose à récupérer.
   */
  if (pid == LIN_PID_TRUMA_INET_BOX) {

    std::array<uint8_t, 8> response =
        this->lin_empty_response_;

    if (this->updates_to_send_.empty() &&
        !this->has_update_to_submit_()) {

      response[0] = 0xFE;
    }

    this->write_lin_answer_(
        response.data(),
        (uint8_t) sizeof(response)
    );

    return true;
  }

  return LinBusProtocol::answer_lin_order_(pid);
}


bool TrumaiNetBoxApp::lin_read_field_by_identifier_(
    uint8_t identifier,
    std::array<uint8_t, 5> *response) {

  /*
   * LIN Product Identification
   */
  if (identifier == 0x00) {

    const auto lin_identifier =
        this->lin_identifier();

    (*response)[0] = lin_identifier[0];
    (*response)[1] = lin_identifier[1];
    (*response)[2] = lin_identifier[2];
    (*response)[3] = lin_identifier[3];

    // Variant
    (*response)[4] = 0x01;

    return true;

  /*
   * Product details displayed by CP Plus
   */
  } else if (identifier == 0x20) {

    const auto lin_identifier =
        this->lin_identifier();

    (*response)[0] = lin_identifier[0];
    (*response)[1] = lin_identifier[1];
    (*response)[2] = lin_identifier[2];

    return true;

  /*
   * Unknown, but required during original iNet init.
   */
  } else if (identifier == 0x22) {

    return true;
  }

  return false;
}


const uint8_t *TrumaiNetBoxApp::lin_multiframe_received(
    const uint8_t *message,
    const uint8_t message_len,
    uint8_t *return_len) {

  static uint8_t response[sizeof(StatusFrame)] = {};

  /*
   * ==========================================================
   * VALIDATION PREFIX
   * ==========================================================
   */

  if (message_len < truma_message_header.size()) {
    return nullptr;
  }

  for (uint8_t i = 1;
       i < truma_message_header.size() - 3;
       i++) {

    if (message[i] != truma_message_header[i] &&
        message[i] != alde_message_header[i]) {

      return nullptr;
    }
  }

  if (message[4] != (uint8_t) this->company_) {

    ESP_LOGI(
        TAG,
        "Switch company to 0x%02x",
        message[4]
    );

    this->company_ =
        (TRUMA_COMPANY) message[4];
  }


  /*
   * ==========================================================
   * READ STATE BUFFER
   * ==========================================================
   */

  if (message[0] == LIN_SID_READ_STATE_BUFFER) {

    memset(response, 0, sizeof(response));

    auto response_frame =
        reinterpret_cast<StatusFrame *>(response);

    /*
     * L'ordre doit rester identique à celui utilisé
     * dans has_update_to_submit_().
     */

    if (this->init_received_.load(
            std::memory_order_relaxed) == 0) {

      ESP_LOGD(
          TAG,
          "Requested read: Sending init"
      );

      status_frame_create_init(
          response_frame,
          return_len,
          this->message_counter++
      );

      return response;

    } else if (this->heater_.has_update()) {

      /*
       * C'est LE log important pour notre essai D4.
       */
      ESP_LOGW(
          TAG,
          "D4 2021: Requested read -> Sending heater update"
      );

      this->heater_.create_update_data(
          response_frame,
          return_len,
          this->message_counter++
      );

      this->update_time_.store(
          0,
          std::memory_order_relaxed
      );

      return response;

    } else if (this->timer_.has_update()) {

      ESP_LOGD(
          TAG,
          "Requested read: Sending timer update"
      );

      this->timer_.create_update_data(
          response_frame,
          return_len,
          this->message_counter++
      );

      this->update_time_.store(
          0,
          std::memory_order_relaxed
      );

      return response;

    } else if (this->airconManual_.has_update()) {

      ESP_LOGD(
          TAG,
          "Requested read: Sending aircon manual update"
      );

      this->airconManual_.create_update_data(
          response_frame,
          return_len,
          this->message_counter++
      );

      this->update_time_.store(
          0,
          std::memory_order_relaxed
      );

      return response;

    } else if (this->airconAuto_.has_update()) {

      ESP_LOGD(
          TAG,
          "Requested read: Sending aircon auto update"
      );

      this->airconAuto_.create_update_data(
          response_frame,
          return_len,
          this->message_counter++
      );

      this->update_time_.store(
          0,
          std::memory_order_relaxed
      );

      return response;

#ifdef USE_TIME

    } else if (this->clock_.has_update()) {

      ESP_LOGD(
          TAG,
          "Requested read: Sending clock update"
      );

      this->clock_.create_update_data(
          response_frame,
          return_len,
          this->message_counter++
      );

      this->update_time_.store(
          0,
          std::memory_order_relaxed
      );

      return response;

#endif

    } else {

      ESP_LOGW(
          TAG,
          "Requested read: CP Plus asks for an update, but I have none."
      );
    }
  }


  /*
   * ==========================================================
   * FILL STATE BUFFER
   * ==========================================================
   */

  if (message_len < sizeof(StatusFrame) &&
      message[0] == LIN_SID_FIll_STATE_BUFFFER) {

    return nullptr;
  }


  /*
   * ==========================================================
   * HEADER VALIDATION
   * ==========================================================
   */

  if (message_len < sizeof(StatusFrameHeader)) {

    ESP_LOGW(
        TAG,
        "Truma frame too short (%u < %u).",
        message_len,
        (unsigned) sizeof(StatusFrameHeader)
    );

    return nullptr;
  }

  auto statusFrame =
      reinterpret_cast<const StatusFrame *>(message);

  auto header =
      &statusFrame->genericHeader;


  /*
   * ==========================================================
   * CHECKSUM
   * ==========================================================
   */

  if (header->checksum !=
          data_checksum(
              &statusFrame->raw[10],
              sizeof(StatusFrame) - 10,
              (0xFF - header->checksum)
          ) ||
      header->header_2 != 'T' ||
      header->header_3 != 0x01) {

    ESP_LOGE(
        TAG,
        "Truma checksum fail."
    );

    return nullptr;
  }


  /*
   * Réponse ACK de base.
   */
  response[0] =
      (header->service_identifier |
       LIN_SID_RESPONSE);

  (*return_len) = 1;


  /*
   * ==========================================================
   * HEATER
   * ==========================================================
   */

  if (header->message_type ==
          STATUS_FRAME_HEATER &&
      header->message_length ==
          sizeof(StatusFrameHeater)) {

    ESP_LOGI(
        TAG,
        "StatusFrameHeater"
    );

    this->heater_.set_status(
        statusFrame->heater
    );

    return response;


  /*
   * ==========================================================
   * AIRCON MANUAL
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_AIRCON_MANUAL &&
      header->message_length ==
          sizeof(StatusFrameAirconManual)) {

    ESP_LOGI(
        TAG,
        "StatusFrameAirconManual"
    );

    this->airconManual_.set_status(
        statusFrame->airconManual
    );

    return response;


  } else if (
      header->message_type ==
          STATUS_FRAME_AIRCON_MANUAL_INIT &&
      header->message_length ==
          sizeof(StatusFrameAirconManualInit)) {

    ESP_LOGI(
        TAG,
        "StatusFrameAirconManualInit"
    );

    return response;


  /*
   * ==========================================================
   * AIRCON AUTO
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_AIRCON_AUTO &&
      header->message_length ==
          sizeof(StatusFrameAirconAuto)) {

    ESP_LOGI(
        TAG,
        "StatusFrameAirconAuto"
    );

    this->airconAuto_.set_status(
        statusFrame->airconAuto
    );

    return response;


  } else if (
      header->message_type ==
          STATUS_FRAME_AIRCON_AUTO_INIT &&
      header->message_length ==
          sizeof(StatusFrameAirconAutoInit)) {

    ESP_LOGI(
        TAG,
        "StatusFrameAirconAutoInit"
    );

    return response;


  /*
   * ==========================================================
   * TIMER
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_TIMER &&
      header->message_length ==
          sizeof(StatusFrameTimer)) {

    ESP_LOGI(
        TAG,
        "StatusFrameTimer"
    );

    this->timer_.set_status(
        statusFrame->timer
    );

    return response;


  /*
   * ==========================================================
   * CLOCK
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_CLOCK &&
      header->message_length ==
          sizeof(StatusFrameClock)) {

    ESP_LOGI(
        TAG,
        "StatusFrameClock"
    );

    this->clock_.set_status(
        statusFrame->clock
    );

    return response;


  /*
   * ==========================================================
   * CONFIG
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_CONFIG &&
      header->message_length ==
          sizeof(StatusFrameConfig)) {

    ESP_LOGI(
        TAG,
        "StatusFrameConfig"
    );

    this->config_.set_status(
        statusFrame->config
    );

    return response;


  /*
   * ==========================================================
   * RESPONSE ACK
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_RESPONSE_ACK &&
      header->message_length ==
          sizeof(StatusFrameResponseAck)) {

    auto data =
        statusFrame->responseAck;

    if (data.error_code !=
        ResponseAckResult::
            RESPONSE_ACK_RESULT_OKAY) {

      ESP_LOGW(
          TAG,
          "StatusFrameResponseAck"
      );

    } else {

      ESP_LOGI(
          TAG,
          "StatusFrameResponseAck"
      );
    }

    ESP_LOGD(
        TAG,
        "StatusFrameResponseAck %02X %s %02X",
        statusFrame->genericHeader.command_counter,
        data.error_code ==
                ResponseAckResult::
                    RESPONSE_ACK_RESULT_OKAY
            ? " OKAY "
            : " FAILED ",
        (uint8_t) data.error_code
    );

    if (data.error_code !=
        ResponseAckResult::
            RESPONSE_ACK_RESULT_OKAY) {

      this->lin_reset_device();
    }

    return response;


  /*
   * ==========================================================
   * DEVICES
   * ==========================================================
   */

  } else if (
      header->message_type ==
          STATUS_FRAME_DEVICES &&
      header->message_length ==
          sizeof(StatusFrameDevice)) {

    ESP_LOGI(
        TAG,
        "StatusFrameDevice"
    );

    auto device =
        statusFrame->device;

    ESP_LOGD(
        TAG,
        "StatusFrameDevice %d/%d - %d.%02d.%02d %04X.%02X (%02X %02X)",
        device.device_id + 1,
        device.device_count,
        device.software_revision[0],
        device.software_revision[1],
        device.software_revision[2],
        device.hardware_revision_major,
        device.hardware_revision_minor,
        device.unknown_2,
        device.unknown_3
    );

    const auto truma_device =
        static_cast<TRUMA_DEVICE>(
            device.software_revision[0]
        );

    {
      bool found_unknown_value = false;

      if (device.unknown_1 != 0x00) {
        found_unknown_value = true;
      }

      if (truma_device !=
              TRUMA_DEVICE::AIRCON_DEVICE &&
          truma_device !=
              TRUMA_DEVICE::HEATER_COMBI4 &&
          truma_device !=
              TRUMA_DEVICE::HEATER_VARIO &&
          truma_device !=
              TRUMA_DEVICE::CPPLUS_COMBI &&
          truma_device !=
              TRUMA_DEVICE::CPPLUS_VARIO &&
          truma_device !=
              TRUMA_DEVICE::HEATER_COMBI6D) {

        found_unknown_value = true;
      }

      if (found_unknown_value) {

        ESP_LOGW(
            TAG,
            "Unknown information in StatusFrameDevice found. Please report."
        );
      }
    }


    /*
     * Premier device = CP Plus
     */
    const auto is_CPPLUSDevice =
        device.device_id == 0;


    if (!is_CPPLUSDevice) {

      /*
       * Premier périphérique après CP Plus = Heater
       */
      if (device.device_id == 1) {

        this->heater_device_.store(
            truma_device,
            std::memory_order_relaxed
        );
      }

      /*
       * Deuxième périphérique après CP Plus = Aircon
       */
      if (device.device_id == 2) {

        this->aircon_device_.store(
            TRUMA_DEVICE::AIRCON_DEVICE,
            std::memory_order_relaxed
        );
      }
    }


    /*
     * Initialisation standard havanti.
     */

    if (device.device_count == 2 &&
        this->heater_device_.load(
            std::memory_order_relaxed) !=
            TRUMA_DEVICE::UNKNOWN) {

      this->init_received_.store(
          micros(),
          std::memory_order_relaxed
      );

    } else if (
        device.device_count == 3 &&
        this->heater_device_.load(
            std::memory_order_relaxed) !=
            TRUMA_DEVICE::UNKNOWN &&
        this->aircon_device_.load(
            std::memory_order_relaxed) !=
            TRUMA_DEVICE::UNKNOWN) {

      this->init_received_.store(
          micros(),
          std::memory_order_relaxed
      );
    }

    return response;


  /*
   * ==========================================================
   * UNKNOWN
   * ==========================================================
   */

  } else {

    ESP_LOGW(
        TAG,
        "Unknown message type %02X",
        header->message_type
    );
  }


  (*return_len) = 0;
  return nullptr;
}


/*
 * ============================================================
 * D4 2021 - UPDATE NOTIFICATION
 * ============================================================
 *
 * Le problème rencontré sur notre Combi D4 :
 *
 * heater_.update_submit() prépare correctement la commande,
 * mais l'initialisation iNet standard ne passe pas jusqu'à
 * init_received_.
 *
 * Dans le code original, tant que init_received_ == 0,
 * heater_.has_update() n'est donc jamais annoncé.
 *
 * Pour le test D4 :
 *
 * si une vraie commande chauffage est en attente,
 * on autorise le mécanisme existant à passer dans l'état
 * "initialisé".
 *
 * IMPORTANT :
 * cette fonction peut être appelée depuis le traitement LIN.
 * PAS DE LOG ici.
 *
 * ============================================================
 */
bool TrumaiNetBoxApp::has_update_to_submit_() {

  const bool heater_update =
      this->heater_.has_update();


  /*
   * ==========================================================
   * D4 BOOTSTRAP
   * ==========================================================
   */

  if (heater_update &&
      this->init_received_.load(
          std::memory_order_relaxed) == 0) {

    this->init_received_.store(
        micros(),
        std::memory_order_relaxed
    );

    this->update_time_.store(
        0,
        std::memory_order_relaxed
    );
  }


  /*
   * ==========================================================
   * INIT REQUEST
   * ==========================================================
   */

  if (this->init_requested_.load(
          std::memory_order_relaxed) == 0) {

    this->init_requested_.store(
        micros(),
        std::memory_order_relaxed
    );

    return true;


  /*
   * ==========================================================
   * WAIT INIT
   * ==========================================================
   */

  } else if (
      this->init_received_.load(
          std::memory_order_relaxed) == 0) {

    auto init_wait_time =
        micros() -
        this->init_requested_.load(
            std::memory_order_relaxed
        );

    if (init_wait_time >
        INIT_RETRY_DELAY_US) {

      this->init_requested_.store(
          micros(),
          std::memory_order_relaxed
      );

      return true;
    }


  /*
   * ==========================================================
   * UPDATE AVAILABLE
   * ==========================================================
   */

  } else if (
      this->airconAuto_.has_update() ||
      this->airconManual_.has_update() ||
      this->clock_.has_update() ||
      heater_update ||
      this->timer_.has_update()) {

    auto update_time_snapshot =
        this->update_time_.load(
            std::memory_order_relaxed
        );


    /*
     * Première notification.
     */
    if (update_time_snapshot == 0) {

      this->update_time_.store(
          micros(),
          std::memory_order_relaxed
      );

      return true;
    }


    /*
     * Retry toutes les 5 secondes.
     */
    auto update_wait_time =
        micros() -
        update_time_snapshot;

    if (update_wait_time >
        UPDATE_RETRY_DELAY_US) {

      this->update_time_.store(
          micros(),
          std::memory_order_relaxed
      );

      return true;
    }
  }


  return false;
}


}  // namespace truma_inetbox
}  // namespace esphome

#include "LinBusProtocol.h"

#include <array>

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusProtocol";

static constexpr uint8_t LIN_TP_FIRST_FRAME_DATA_LEN = 5;
static constexpr uint8_t LIN_TP_CONSECUTIVE_FRAME_DATA_LEN = 6;

#define LIN_NAD_BROADCAST 0x7F

#define LIN_SID_RESPONSE 0x40

#define LIN_SID_ASSIGN_NAD 0xB0
#define LIN_SID_ASSIGN_NAD_RESPONSE \
  (LIN_SID_ASSIGN_NAD | LIN_SID_RESPONSE)

#define LIN_SID_READ_BY_IDENTIFIER 0xB2
#define LIN_SID_READ_BY_IDENTIFIER_RESPONSE \
  (LIN_SID_READ_BY_IDENTIFIER | LIN_SID_RESPONSE)

#define LIN_SID_HEARTBEAT 0xB9
#define LIN_SID_HEARTBEAT_RESPONSE \
  (LIN_SID_HEARTBEAT | LIN_SID_RESPONSE)


void LinBusProtocol::lin_reset_device() {
  ESP_LOGW(TAG, "LIN RESET: vidage de la file TX");

  while (!this->updates_to_send_.empty()) {
    this->updates_to_send_.pop();
  }
}


bool LinBusProtocol::answer_lin_order_(const uint8_t pid) {
  if (pid != DIAGNOSTIC_FRAME_SLAVE) {
    return false;
  }

  if (this->updates_to_send_.empty()) {
    return false;
  }

  auto response = this->updates_to_send_.front();
  this->updates_to_send_.pop();

  ESP_LOGW(
      TAG,
      "D4 TX PID3D NAD=%02X : %s",
      this->lin_node_address_,
      format_hex_pretty(
          response.data(),
          response.size()
      ).c_str()
  );

  this->write_lin_answer_(
      response.data(),
      (uint8_t) response.size()
  );

  return true;
}


void LinBusProtocol::lin_message_received_(
    const uint8_t pid,
    const uint8_t *message,
    uint8_t length) {

  if (pid == DIAGNOSTIC_FRAME_MASTER) {

    if (length < 3) {
      ESP_LOGE(
          TAG,
          "LIN Protocol issue: Diagnostic frame too short (%u bytes).",
          length
      );
      return;
    }

    bool my_node_address =
        message[0] == this->lin_node_address_;

    bool broadcast_address =
        message[0] == LIN_NAD_BROADCAST;

    /*
     * On logue les requêtes de diagnostic importantes,
     * même lorsqu'elles ne nous sont pas destinées.
     *
     * Cela permet de voir :
     *   - le NAD demandé par le CP Plus
     *   - les B2
     *   - les B0
     *   - l'identifiant recherché
     */
    uint8_t pci = message[1];

    if ((pci & 0xF0) == 0x00 && length >= 3) {
      uint8_t sid = message[2];

      if (sid == LIN_SID_READ_BY_IDENTIFIER ||
          sid == LIN_SID_ASSIGN_NAD ||
          sid == LIN_SID_HEARTBEAT) {

        ESP_LOGW(
            TAG,
            "D4 RX PID3C NAD=%02X CURRENT_NAD=%02X SID=%02X : %s",
            message[0],
            this->lin_node_address_,
            sid,
            format_hex_pretty(message, length).c_str()
        );
      }
    }

    /*
     * IMPORTANT :
     * on ne traite que :
     *
     *   - notre NAD
     *   - le broadcast 0x7F
     *
     * Une requête destinée au NAD d'un autre appareil
     * est uniquement observée.
     */
    if (!my_node_address && !broadcast_address) {
      return;
    }

    uint8_t protocol_control_information = message[1];

    if ((protocol_control_information & 0xF0) == 0x00) {

      // Fin d'une éventuelle réception multi-frame précédente.
      this->multi_pdu_message_expected_size_ = 0;
      this->multi_pdu_message_len_ = 0;
      this->multi_pdu_message_frame_counter_ = 0;

      this->lin_msg_diag_single_(message, length);

    } else if (
        (protocol_control_information & 0xF0) == 0x10) {

      this->lin_msg_diag_first_(message, length);

    } else if (
        (protocol_control_information & 0xF0) == 0x20) {

      if (this->lin_msg_diag_consecutive_(
              message,
              length)) {

        this->lin_msg_diag_multi_();
      }
    }

    return;
  }

  if (pid == this->lin_node_address_) {
    ESP_LOGW(
        TAG,
        "Unhandled message for NAD %02X.",
        this->lin_node_address_
    );
  }
}


bool LinBusProtocol::is_matching_identifier_(
    const uint8_t *message) {

  auto identifier = this->lin_identifier();

  bool match =
      message[0] == identifier[0] &&
      message[1] == identifier[1] &&
      message[2] == identifier[2] &&
      message[3] == identifier[3];

  ESP_LOGW(
      TAG,
      "D4 IDENTIFIER RX=%02X.%02X.%02X.%02X "
      "LOCAL=%02X.%02X.%02X.%02X MATCH=%s",
      message[0],
      message[1],
      message[2],
      message[3],
      identifier[0],
      identifier[1],
      identifier[2],
      identifier[3],
      match ? "YES" : "NO"
  );

  return match;
}


void LinBusProtocol::lin_msg_diag_single_(
    const uint8_t *message,
    uint8_t length) {

  bool my_node_address =
      message[0] == this->lin_node_address_;

  bool broadcast_address =
      message[0] == LIN_NAD_BROADCAST;

  uint8_t message_length = message[1];
  uint8_t service_identifier = message[2];

  if (message_length > 6) {
    ESP_LOGE(
        TAG,
        "LIN Protocol issue: Single frame message too long."
    );
    return;
  }


  /*
   * ==========================================================
   * READ BY IDENTIFIER — B2
   * ==========================================================
   */

  if (service_identifier ==
          LIN_SID_READ_BY_IDENTIFIER &&
      message_length == 6) {

    ESP_LOGW(
        TAG,
        "D4 B2: NAD=%02X identifier=%02X",
        message[0],
        message[3]
    );

    if (this->is_matching_identifier_(&message[4])) {

      ESP_LOGW(
          TAG,
          "D4 B2 MATCH: cette requete concerne notre iNet Box"
      );

      uint8_t identifier = message[3];

      std::array<uint8_t, 8> response =
          this->lin_empty_response_;

      response[0] = this->lin_node_address_;

      std::array<uint8_t, 5> identifier_response = {};

      if (this->lin_read_field_by_identifier_(
              identifier,
              &identifier_response)) {

        response[1] = 6;

        response[2] =
            LIN_SID_READ_BY_IDENTIFIER_RESPONSE;

        auto iterator = response.begin();
        std::advance(iterator, 3);

        std::copy(
            identifier_response.data(),
            identifier_response.data() +
                identifier_response.size(),
            iterator
        );

        ESP_LOGW(
            TAG,
            "D4 B2 RESPONSE positive : %s",
            format_hex_pretty(
                response.data(),
                response.size()
            ).c_str()
        );

      } else {

        response[1] = 3;
        response[2] = 0x7F;
        response[3] =
            LIN_SID_READ_BY_IDENTIFIER;
        response[4] = 0x12;

        ESP_LOGW(
            TAG,
            "D4 B2 RESPONSE negative : %s",
            format_hex_pretty(
                response.data(),
                response.size()
            ).c_str()
        );
      }

      this->prepare_update_msg_(response);

    } else {

      ESP_LOGW(
          TAG,
          "D4 B2 IGNORE: identifiant different"
      );
    }

    return;
  }


  /*
   * ==========================================================
   * HEARTBEAT — B9
   * ==========================================================
   */

  if (my_node_address &&
      service_identifier ==
          LIN_SID_HEARTBEAT &&
      message_length >= 5) {

    ESP_LOGW(
        TAG,
        "D4 HEARTBEAT NAD=%02X",
        this->lin_node_address_
    );

    std::array<uint8_t, 8> response =
        this->lin_empty_response_;

    response[0] = this->lin_node_address_;
    response[1] = 2;
    response[2] =
        LIN_SID_HEARTBEAT_RESPONSE;
    response[3] = 0x00;

    this->prepare_update_msg_(response);

    this->lin_heartbeat();

    return;
  }


  /*
   * ==========================================================
   * ASSIGN NAD — B0
   * ==========================================================
   */

  if (broadcast_address &&
      service_identifier ==
          LIN_SID_ASSIGN_NAD &&
      message_length == 6) {

    ESP_LOGW(
        TAG,
        "D4 B0 ASSIGN NAD recu : %s",
        format_hex_pretty(
            message,
            length
        ).c_str()
    );

    if (this->is_matching_identifier_(
            &message[3])) {

      uint8_t old_nad =
          this->lin_node_address_;

      uint8_t new_nad =
          message[7];

      ESP_LOGW(
          TAG,
          "=============================================="
      );

      ESP_LOGW(
          TAG,
          "D4 NAD ASSIGNMENT MATCH"
      );

      ESP_LOGW(
          TAG,
          "D4 NAD : %02X -> %02X",
          old_nad,
          new_nad
      );

      ESP_LOGW(
          TAG,
          "=============================================="
      );

      /*
       * La réponse doit encore utiliser
       * l'ancien NAD.
       */

      std::array<uint8_t, 8> response =
          this->lin_empty_response_;

      response[0] = old_nad;
      response[1] = 1;

      response[2] =
          LIN_SID_ASSIGN_NAD_RESPONSE;

      this->prepare_update_msg_(response);

      /*
       * Puis on adopte le nouveau NAD.
       */
      this->lin_node_address_ = new_nad;

    } else {

      ESP_LOGW(
          TAG,
          "D4 B0 IGNORE: identifiant different"
      );
    }

    return;
  }


  /*
   * ==========================================================
   * AUTRES SID
   * ==========================================================
   */

  if (my_node_address) {

    ESP_LOGD(
        TAG,
        "SID %02X MY NAD=%02X - %s - Unhandled",
        service_identifier,
        this->lin_node_address_,
        format_hex_pretty(
            message,
            length
        ).c_str()
    );

  } else if (broadcast_address) {

    ESP_LOGD(
        TAG,
        "SID %02X BC - %s - Unhandled",
        service_identifier,
        format_hex_pretty(
            message,
            length
        ).c_str()
    );
  }
}


void LinBusProtocol::lin_msg_diag_first_(
    const uint8_t *message,
    uint8_t length) {

  uint8_t protocol_control_information =
      message[1];

  uint16_t message_length =
      ((protocol_control_information & 0x0F)
       << 8) +
      message[2];

  if (message_length < 7) {

    ESP_LOGE(
        TAG,
        "LIN Protocol issue: Multi frame message too short."
    );

    return;
  }

  if (message_length >
      sizeof(this->multi_pdu_message_)) {

    ESP_LOGE(
        TAG,
        "LIN Protocol issue: Multi frame message too long."
    );

    return;
  }

  this->multi_pdu_message_expected_size_ =
      message_length;

  this->multi_pdu_message_len_ = 0;

  this->multi_pdu_message_frame_counter_ = 1;

  for (size_t i = 3; i < 8; i++) {

    this->multi_pdu_message_[
        this->multi_pdu_message_len_++
    ] = message[i];
  }
}


bool LinBusProtocol::lin_msg_diag_consecutive_(
    const uint8_t *message,
    uint8_t length) {

  if (this->multi_pdu_message_len_ >=
      this->multi_pdu_message_expected_size_) {

    return false;
  }

  uint8_t protocol_control_information =
      message[1];

  uint8_t frame_counter =
      protocol_control_information & 0x0F;

  if (frame_counter !=
      this->multi_pdu_message_frame_counter_) {

    return false;
  }

  this->multi_pdu_message_frame_counter_++;

  if (this->multi_pdu_message_frame_counter_ >
      0x0F) {

    this->multi_pdu_message_frame_counter_ =
        0x00;
  }

  for (uint8_t i = 2; i < 8; i++) {

    if (this->multi_pdu_message_len_ <
        this->multi_pdu_message_expected_size_) {

      this->multi_pdu_message_[
          this->multi_pdu_message_len_++
      ] = message[i];
    }
  }

  return
      this->multi_pdu_message_len_ ==
      this->multi_pdu_message_expected_size_;
}


void LinBusProtocol::lin_msg_diag_multi_() {

  ESP_LOGD(
      TAG,
      "Multi package request %s",
      format_hex_pretty(
          this->multi_pdu_message_,
          this->multi_pdu_message_len_
      ).c_str()
  );

  uint8_t answer_len = 0;

  auto answer =
      this->lin_multiframe_received(
          this->multi_pdu_message_,
          this->multi_pdu_message_len_,
          &answer_len
      );

  if (answer_len == 0) {
    return;
  }

  ESP_LOGD(
      TAG,
      "Multi package response %s",
      format_hex_pretty(
          answer,
          answer_len
      ).c_str()
  );

  std::array<uint8_t, 8> response =
      this->lin_empty_response_;


  /*
   * ==========================================================
   * SINGLE FRAME RESPONSE
   * ==========================================================
   */

  if (answer_len <= 6) {

    response[0] =
        this->lin_node_address_;

    response[1] =
        answer_len;

    response[2] =
        answer[0] | LIN_SID_RESPONSE;

    for (uint8_t i = 1;
         i < answer_len;
         i++) {

      response[i + 2] =
          answer[i];
    }

    ESP_LOGW(
        TAG,
        "D4 APP RESPONSE SF NAD=%02X : %s",
        this->lin_node_address_,
        format_hex_pretty(
            response.data(),
            response.size()
        ).c_str()
    );

    this->prepare_update_msg_(response);

    return;
  }


  /*
   * ==========================================================
   * MULTI FRAME RESPONSE
   * ==========================================================
   */

  response[0] =
      this->lin_node_address_;

  response[1] = 0x10;

  response[2] =
      answer_len;

  response[3] =
      answer[0] | LIN_SID_RESPONSE;

  for (uint8_t i = 1;
       i < LIN_TP_FIRST_FRAME_DATA_LEN;
       i++) {

    response[i + 3] =
        answer[i];
  }

  ESP_LOGW(
      TAG,
      "D4 APP RESPONSE FF NAD=%02X : %s",
      this->lin_node_address_,
      format_hex_pretty(
          response.data(),
          response.size()
      ).c_str()
  );

  this->prepare_update_msg_(response);


  uint16_t answer_position =
      LIN_TP_FIRST_FRAME_DATA_LEN;

  uint8_t answer_frame_counter = 0;

  while (answer_position < answer_len) {

    response =
        this->lin_empty_response_;

    response[0] =
        this->lin_node_address_;

    response[1] =
        ((answer_frame_counter + 1) & 0x0F)
        | 0x20;

    for (uint8_t i = 0;
         i <
         LIN_TP_CONSECUTIVE_FRAME_DATA_LEN;
         i++) {

      if (answer_position <
          answer_len) {

        response[i + 2] =
            answer[answer_position++];
      }
    }

    ESP_LOGW(
        TAG,
        "D4 APP RESPONSE CF%u NAD=%02X : %s",
        answer_frame_counter + 1,
        this->lin_node_address_,
        format_hex_pretty(
            response.data(),
            response.size()
        ).c_str()
    );

    this->prepare_update_msg_(response);

    answer_frame_counter++;
  }
}


}  // namespace truma_inetbox
}  // namespace esphome

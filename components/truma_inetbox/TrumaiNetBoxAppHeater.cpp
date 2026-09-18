#include "TrumaiNetBoxAppHeater.h"
#include "TrumaStatusFrameBuilder.h"
#include "esphome/core/log.h"
#include "helpers.h"
#include "TrumaiNetBoxApp.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.TrumaiNetBoxAppHeater";

StatusFrameHeaterResponse *TrumaiNetBoxAppHeater::update_prepare() {
  // An update is currently going on.
  if (this->update_status_prepared_ || this->update_status_stale_) {
    return &this->update_status_;
  }

  // Prepare status heater response.
  this->update_status_ = {};
  this->update_status_.target_temp_room = this->data_.target_temp_room;
  this->update_status_.heating_mode = this->data_.heating_mode;
  this->update_status_.el_power_level_a = this->data_.el_power_level_a;
  this->update_status_.target_temp_water = this->data_.target_temp_water;
  this->update_status_.el_power_level_b = this->data_.el_power_level_b;
  this->update_status_.energy_mix_a = this->data_.energy_mix_a;
  this->update_status_.energy_mix_b = this->data_.energy_mix_b;

  this->update_status_prepared_ = true;

  return &this->update_status_;
}


void TrumaiNetBoxAppHeater::create_update_data(
    StatusFrame *response,
    uint8_t *response_len,
    uint8_t command_counter) {

  status_frame_create_empty(
      response,
      STATUS_FRAME_HEATER_RESPONSE,
      sizeof(StatusFrameHeaterResponse),
      command_counter);

  response->heaterResponse.target_temp_room =
      this->update_status_.target_temp_room;

  response->heaterResponse.heating_mode =
      this->update_status_.heating_mode;

  response->heaterResponse.target_temp_water =
      this->update_status_.target_temp_water;

  response->heaterResponse.energy_mix_a =
      this->update_status_.energy_mix_a;

  response->heaterResponse.energy_mix_b =
      this->update_status_.energy_mix_b;

  response->heaterResponse.el_power_level_a =
      this->update_status_.el_power_level_a;

  response->heaterResponse.el_power_level_b =
      this->update_status_.el_power_level_b;

  status_frame_calculate_checksum(response);

  (*response_len) =
      sizeof(StatusFrameHeader) +
      sizeof(StatusFrameHeaterResponse);

  TrumaStausFrameResponseStorage<
      StatusFrameHeater,
      StatusFrameHeaterResponse>::update_submitted();
}


void TrumaiNetBoxAppHeater::dump_data() const {

  ESP_LOGD(
      TAG,
      "StatusFrameHeater room: %.1f°C mode: %u water: %.1f°C "
      "energy_mix: %u el_power: %u W status: %s",

      temp_code_to_decimal(this->data_.target_temp_room),
      (uint16_t) this->data_.heating_mode,
      temp_code_to_decimal(this->data_.target_temp_water),
      (uint8_t) this->data_.energy_mix_a,
      (uint16_t) this->data_.el_power_level_a,
      operating_status_to_str(
          this->data_.operating_status).c_str());

  if (
      this->data_.error_code_low != 0 ||
      this->data_.error_code_high != 0) {

    ESP_LOGW(
        TAG,
        "StatusFrameHeater error_code: 0x%02X 0x%02X",
        this->data_.error_code_low,
        this->data_.error_code_high);
  }
}


/*
 * ============================================================
 * PATCH TRUMA COMBI D4 2021
 * ============================================================
 *
 * Version originale :
 *
 * return Storage::can_update() &&
 *        parent_->get_heater_device() != TRUMA_DEVICE::UNKNOWN;
 *
 * Sur notre Combi D4 2021, la couche LIN fonctionne mais
 * l'identification haut niveau du chauffage reste UNKNOWN.
 *
 * Cela provoque :
 *
 *     Cannot update Truma.
 *
 * On conserve la sécurité importante :
 *
 *     Storage::can_update()
 *
 * donc aucune commande ne peut être construite tant qu'un
 * StatusFrameHeater valide n'a pas été reçu.
 *
 * On retire uniquement le test get_heater_device() != UNKNOWN.
 * ============================================================
 */

bool TrumaiNetBoxAppHeater::can_update() {

  bool status_valid =
      TrumaStausFrameResponseStorage<
          StatusFrameHeater,
          StatusFrameHeaterResponse>::can_update();

  if (!status_valid) {

    ESP_LOGW(
        TAG,
        "D4 2021 PATCH: StatusFrameHeater non valide, "
        "ecriture refusee.");

    return false;
  }

  if (this->parent_->get_heater_device() == TRUMA_DEVICE::UNKNOWN) {

    ESP_LOGW(
        TAG,
        "D4 2021 PATCH: heater_device UNKNOWN, "
        "ecriture autorisee grace au StatusFrameHeater valide.");
  }

  return true;
}


bool TrumaiNetBoxAppHeater::action_heater_room(
    uint8_t temperature,
    HeatingMode mode) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }

  auto heater = this->update_prepare();


  ESP_LOGW(
      TAG,
      "D4 2021 PATCH: demande chauffage %u C, mode=%u",
      temperature,
      (uint16_t) mode);


  heater->target_temp_room =
      decimal_to_room_temp(temperature);


  // Ensure heating_mode and energy_mix_a are set.
  if (
      heater->target_temp_room ==
      TargetTemp::TARGET_TEMP_OFF) {

    heater->heating_mode =
        HeatingMode::HEATING_MODE_OFF;

  } else {

    if (
        this->parent_->get_heater_device() ==
        TRUMA_DEVICE::HEATER_VARIO) {

      if (
          mode ==
              HeatingMode::HEATING_MODE_VARIO_HEAT_NIGHT ||
          mode ==
              HeatingMode::HEATING_MODE_VARIO_HEAT_AUTO ||
          mode ==
              HeatingMode::HEATING_MODE_BOOST) {

        heater->heating_mode = mode;

      } else if (
          heater->heating_mode ==
          HeatingMode::HEATING_MODE_OFF) {

        heater->heating_mode =
            HeatingMode::HEATING_MODE_VARIO_HEAT_AUTO;
      }

    } else {

      /*
       * COMBI
       *
       * IMPORTANT pour notre D4 :
       * même si get_heater_device() == UNKNOWN,
       * on utilise ici le comportement COMBI.
       */

      if (
          mode == HeatingMode::HEATING_MODE_ECO ||
          mode == HeatingMode::HEATING_MODE_HIGH ||
          mode == HeatingMode::HEATING_MODE_BOOST) {

        heater->heating_mode = mode;

      } else if (
          heater->heating_mode ==
          HeatingMode::HEATING_MODE_OFF) {

        heater->heating_mode =
            HeatingMode::HEATING_MODE_ECO;
      }
    }
  }


  /*
   * On conserve POUR CE PREMIER TEST le comportement original.
   *
   * Nous voulons d'abord vérifier si la transaction iNet
   * est maintenant générée.
   *
   * On modifiera le traitement Diesel uniquement si les logs
   * montrent que c'est nécessaire.
   */

  if (
      heater->energy_mix_a ==
      EnergyMix::ENERGY_MIX_NONE) {

    heater->energy_mix_a =
        EnergyMix::ENERGY_MIX_GAS;
  }


  ESP_LOGW(
      TAG,
      "D4 2021 PATCH: commande preparee "
      "target_room=%u heating_mode=%u energy_mix=%u",

      (uint16_t) heater->target_temp_room,
      (uint16_t) heater->heating_mode,
      (uint16_t) heater->energy_mix_a);


  this->update_submit();


  ESP_LOGW(
      TAG,
      "D4 2021 PATCH: update_submit() effectue.");


  return true;
}


bool TrumaiNetBoxAppHeater::action_heater_water(
    uint8_t temperature) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }

  auto heater = this->update_prepare();

  heater->target_temp_water =
      decimal_to_water_temp(temperature);


  if (
      heater->target_temp_water !=
          TargetTemp::TARGET_TEMP_OFF &&
      heater->energy_mix_a ==
          EnergyMix::ENERGY_MIX_NONE) {

    heater->energy_mix_a =
        EnergyMix::ENERGY_MIX_GAS;
  }


  this->update_submit();

  return true;
}


bool TrumaiNetBoxAppHeater::action_heater_water(
    TargetTemp temperature) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }

  auto heater = this->update_prepare();


  if (
      temperature ==
          TargetTemp::TARGET_TEMP_WATER_ECO ||
      temperature ==
          TargetTemp::TARGET_TEMP_WATER_HIGH ||
      temperature ==
          TargetTemp::TARGET_TEMP_WATER_BOOST) {

    heater->target_temp_water =
        temperature;

  } else {

    heater->target_temp_water =
        TargetTemp::TARGET_TEMP_OFF;
  }


  if (
      heater->target_temp_water !=
          TargetTemp::TARGET_TEMP_OFF &&
      heater->energy_mix_a ==
          EnergyMix::ENERGY_MIX_NONE) {

    heater->energy_mix_a =
        EnergyMix::ENERGY_MIX_GAS;
  }


  this->update_submit();

  return true;
}


bool TrumaiNetBoxAppHeater::action_heater_electric_power_level(
    uint16_t value) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }

  auto heater = this->update_prepare();


  heater->el_power_level_a =
      decimal_to_el_power_level(value);


  if (
      heater->el_power_level_a !=
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0) {

    if (
        heater->energy_mix_a !=
            EnergyMix::ENERGY_MIX_MIX &&
        heater->energy_mix_a !=
            EnergyMix::ENERGY_MIX_ELECTRICITY) {

      heater->energy_mix_a =
          EnergyMix::ENERGY_MIX_MIX;
    }

  } else {

    heater->energy_mix_a =
        EnergyMix::ENERGY_MIX_GAS;
  }


  this->update_submit();

  return true;
}


bool TrumaiNetBoxAppHeater::action_heater_energy_mix(
    EnergyMix energy_mix,
    ElectricPowerLevel el_power_level) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }

  auto heater = this->update_prepare();


  if (
      el_power_level ==
          ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0 ||
      el_power_level ==
          ElectricPowerLevel::ELECTRIC_POWER_LEVEL_900 ||
      el_power_level ==
          ElectricPowerLevel::ELECTRIC_POWER_LEVEL_1800) {

    heater->el_power_level_a =
        el_power_level;
  }


  if (
      energy_mix ==
      EnergyMix::ENERGY_MIX_GAS) {

    heater->energy_mix_a =
        energy_mix;

    heater->el_power_level_a =
        ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;


  } else if (
      energy_mix ==
          EnergyMix::ENERGY_MIX_MIX ||
      energy_mix ==
          EnergyMix::ENERGY_MIX_ELECTRICITY) {

    heater->energy_mix_a =
        energy_mix;


    if (
        heater->el_power_level_a ==
        ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0) {

      heater->el_power_level_a =
          ElectricPowerLevel::ELECTRIC_POWER_LEVEL_900;
    }
  }


  /*
   * Consistency guard from original implementation.
   */

  if (
      heater->el_power_level_a !=
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0) {

    if (
        heater->energy_mix_a !=
            EnergyMix::ENERGY_MIX_MIX &&
        heater->energy_mix_a !=
            EnergyMix::ENERGY_MIX_ELECTRICITY) {

      heater->energy_mix_a =
          EnergyMix::ENERGY_MIX_MIX;
    }

  } else {

    heater->energy_mix_a =
        EnergyMix::ENERGY_MIX_GAS;
  }


  this->update_submit();

  return true;
}


}  // namespace truma_inetbox
}  // namespace esphome

#include "TrumaiNetBoxAppHeater.h"
#include "TrumaStatusFrameBuilder.h"
#include "esphome/core/log.h"
#include "helpers.h"
#include "TrumaiNetBoxApp.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.TrumaiNetBoxAppHeater";


/*
 * ============================================================
 * TRUMA COMBI D4 2021
 *
 * Le D4 2021 utilisé ici communique correctement sur le bus LIN,
 * mais la version actuelle de la couche applicative havanti ne
 * reconnaît pas son STATUS_FRAME_HEATER.
 *
 * Conséquence :
 *
 *   data_valid_ == false
 *   get_heater_device() == UNKNOWN
 *
 * alors que les PID LIN réels sont parfaitement présents.
 *
 * Pour permettre le développement de l'écriture sans utiliser
 * une structure data_ invalide, on crée un état de départ sûr.
 *
 * IMPORTANT :
 * ce bootstrap concerne uniquement le cas où havanti n'a pas
 * encore reçu un StatusFrameHeater exploitable.
 * ============================================================
 */


StatusFrameHeaterResponse *TrumaiNetBoxAppHeater::update_prepare() {

  // Une mise à jour est déjà préparée.
  if (this->update_status_prepared_ ||
      this->update_status_stale_) {

    return &this->update_status_;
  }


  this->update_status_ = {};


  /*
   * ----------------------------------------------------------
   * CAS NORMAL
   *
   * Si havanti possède réellement un StatusFrameHeater valide,
   * on conserve exactement son comportement normal.
   * ----------------------------------------------------------
   */

  bool status_valid =
      TrumaStausFrameResponseStorage<
          StatusFrameHeater,
          StatusFrameHeaterResponse>::can_update();


  if (status_valid) {

    ESP_LOGI(
        TAG,
        "D4 2021: StatusFrameHeater valide - "
        "utilisation des valeurs recues."
    );


    this->update_status_.target_temp_room =
        this->data_.target_temp_room;

    this->update_status_.heating_mode =
        this->data_.heating_mode;

    this->update_status_.el_power_level_a =
        this->data_.el_power_level_a;

    this->update_status_.target_temp_water =
        this->data_.target_temp_water;

    this->update_status_.el_power_level_b =
        this->data_.el_power_level_b;

    this->update_status_.energy_mix_a =
        this->data_.energy_mix_a;

    this->update_status_.energy_mix_b =
        this->data_.energy_mix_b;
  }


  /*
   * ----------------------------------------------------------
   * BOOTSTRAP D4 2021
   *
   * Aucun StatusFrameHeater reconnu.
   *
   * NE PAS copier this->data_.
   *
   * On construit volontairement un état minimal :
   *
   * chauffage = OFF
   * eau        = OFF
   * électrique = 0 W
   *
   * ENERGY_MIX_GAS est conservé ici car c'est la valeur utilisée
   * par la bibliothèque pour la source combustible du Combi.
   *
   * Nous ne supposons PAS encore qu'une valeur enum DIESEL
   * différente existe sur le protocole.
   * ----------------------------------------------------------
   */

  else {

    ESP_LOGW(
        TAG,
        "D4 2021 BOOTSTRAP: aucun StatusFrameHeater valide."
    );

    ESP_LOGW(
        TAG,
        "D4 2021 BOOTSTRAP: creation d'un etat chauffage minimal."
    );


    this->update_status_.target_temp_room =
        TargetTemp::TARGET_TEMP_OFF;

    this->update_status_.heating_mode =
        HeatingMode::HEATING_MODE_OFF;


    this->update_status_.el_power_level_a =
        ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;

    this->update_status_.el_power_level_b =
        ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;


    this->update_status_.target_temp_water =
        TargetTemp::TARGET_TEMP_OFF;


    this->update_status_.energy_mix_a =
        EnergyMix::ENERGY_MIX_GAS;

    this->update_status_.energy_mix_b =
        EnergyMix::ENERGY_MIX_GAS;
  }


  this->update_status_prepared_ = true;

  return &this->update_status_;
}



void TrumaiNetBoxAppHeater::create_update_data(
    StatusFrame *response,
    uint8_t *response_len,
    uint8_t command_counter) {

  ESP_LOGW(
      TAG,
      "D4 2021: construction STATUS_FRAME_HEATER_RESPONSE "
      "counter=%u",
      command_counter
  );


  status_frame_create_empty(
      response,
      STATUS_FRAME_HEATER_RESPONSE,
      sizeof(StatusFrameHeaterResponse),
      command_counter
  );


  response->heaterResponse.target_temp_room =
      this->update_status_.target_temp_room;

  response->heaterResponse.heating_mode =
      this->update_status_.heating_mode;

  response->heaterResponse.el_power_level_a =
      this->update_status_.el_power_level_a;

  response->heaterResponse.target_temp_water =
      this->update_status_.target_temp_water;

  response->heaterResponse.el_power_level_b =
      this->update_status_.el_power_level_b;

  response->heaterResponse.energy_mix_a =
      this->update_status_.energy_mix_a;

  response->heaterResponse.energy_mix_b =
      this->update_status_.energy_mix_b;


  ESP_LOGW(
      TAG,
      "D4 2021 RESPONSE: room=%u mode=%u "
      "elA=%u water=%u elB=%u mixA=%u mixB=%u",

      (uint16_t)this->update_status_.target_temp_room,
      (uint16_t)this->update_status_.heating_mode,
      (uint16_t)this->update_status_.el_power_level_a,
      (uint16_t)this->update_status_.target_temp_water,
      (uint16_t)this->update_status_.el_power_level_b,
      (uint16_t)this->update_status_.energy_mix_a,
      (uint16_t)this->update_status_.energy_mix_b
  );


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
      (uint16_t)this->data_.heating_mode,
      temp_code_to_decimal(this->data_.target_temp_water),
      (uint8_t)this->data_.energy_mix_a,
      (uint16_t)this->data_.el_power_level_a,
      operating_status_to_str(
          this->data_.operating_status
      ).c_str()
  );


  if (this->data_.error_code_low != 0 ||
      this->data_.error_code_high != 0) {

    ESP_LOGW(
        TAG,
        "StatusFrameHeater error_code: 0x%02X 0x%02X",
        this->data_.error_code_low,
        this->data_.error_code_high
    );
  }
}



/*
 * ============================================================
 * D4 2021
 *
 * L'absence de StatusFrameHeater reconnu ne doit plus empêcher
 * la préparation d'une commande.
 *
 * update_prepare() sait maintenant gérer ce cas sans lire
 * this->data_.
 * ============================================================
 */

bool TrumaiNetBoxAppHeater::can_update() {

  bool status_valid =
      TrumaStausFrameResponseStorage<
          StatusFrameHeater,
          StatusFrameHeaterResponse>::can_update();


  if (status_valid) {

    ESP_LOGI(
        TAG,
        "D4 2021: StatusFrameHeater valide."
    );

    return true;
  }


  ESP_LOGW(
      TAG,
      "D4 2021 BOOTSTRAP: StatusFrameHeater absent - "
      "ecriture autorisee avec valeurs initialisees."
  );


  return true;
}



bool TrumaiNetBoxAppHeater::action_heater_room(
    uint8_t temperature,
    HeatingMode mode) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }


  auto heater =
      this->update_prepare();


  ESP_LOGW(
      TAG,
      "=============================================="
  );

  ESP_LOGW(
      TAG,
      "D4 2021: DEMANDE CHAUFFAGE %u C",
      temperature
  );

  ESP_LOGW(
      TAG,
      "D4 2021: heating_mode demande = %u",
      (uint16_t)mode
  );


  heater->target_temp_room =
      decimal_to_room_temp(temperature);


  if (heater->target_temp_room ==
      TargetTemp::TARGET_TEMP_OFF) {

    heater->heating_mode =
        HeatingMode::HEATING_MODE_OFF;
  }

  else {

    /*
     * Notre appareil est un COMBI D4.
     *
     * Même si get_heater_device() reste UNKNOWN,
     * on applique donc ici les modes COMBI.
     */

    if (mode == HeatingMode::HEATING_MODE_ECO ||
        mode == HeatingMode::HEATING_MODE_HIGH ||
        mode == HeatingMode::HEATING_MODE_BOOST) {

      heater->heating_mode = mode;
    }

    else {

      heater->heating_mode =
          HeatingMode::HEATING_MODE_ECO;
    }
  }


  /*
   * Pas de puissance électrique sur notre D4.
   */

  heater->el_power_level_a =
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;

  heater->el_power_level_b =
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;


  /*
   * Pour le premier test, on conserve la représentation
   * "combustible" utilisée par la bibliothèque.
   */

  heater->energy_mix_a =
      EnergyMix::ENERGY_MIX_GAS;

  heater->energy_mix_b =
      EnergyMix::ENERGY_MIX_GAS;


  ESP_LOGW(
      TAG,
      "D4 2021 PREPARE: room=%u mode=%u "
      "water=%u mixA=%u",

      (uint16_t)heater->target_temp_room,
      (uint16_t)heater->heating_mode,
      (uint16_t)heater->target_temp_water,
      (uint16_t)heater->energy_mix_a
  );


  this->update_submit();


  ESP_LOGW(
      TAG,
      "D4 2021: update_submit() OK"
  );

  ESP_LOGW(
      TAG,
      "=============================================="
  );


  return true;
}



bool TrumaiNetBoxAppHeater::action_heater_water(
    uint8_t temperature) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }


  auto heater =
      this->update_prepare();


  heater->target_temp_water =
      decimal_to_water_temp(temperature);


  heater->el_power_level_a =
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;

  heater->el_power_level_b =
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;


  heater->energy_mix_a =
      EnergyMix::ENERGY_MIX_GAS;

  heater->energy_mix_b =
      EnergyMix::ENERGY_MIX_GAS;


  this->update_submit();

  return true;
}



bool TrumaiNetBoxAppHeater::action_heater_water(
    TargetTemp temperature) {

  if (!this->can_update()) {

    ESP_LOGW(TAG, "Cannot update Truma.");

    return false;
  }


  auto heater =
      this->update_prepare();


  if (temperature ==
          TargetTemp::TARGET_TEMP_WATER_ECO ||
      temperature ==
          TargetTemp::TARGET_TEMP_WATER_HIGH ||
      temperature ==
          TargetTemp::TARGET_TEMP_WATER_BOOST) {

    heater->target_temp_water =
        temperature;
  }

  else {

    heater->target_temp_water =
        TargetTemp::TARGET_TEMP_OFF;
  }


  heater->el_power_level_a =
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;

  heater->el_power_level_b =
      ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0;


  heater->energy_mix_a =
      EnergyMix::ENERGY_MIX_GAS;

  heater->energy_mix_b =
      EnergyMix::ENERGY_MIX_GAS;


  this->update_submit();

  return true;
}



bool TrumaiNetBoxAppHeater::action_heater_electric_power_level(
    uint16_t value) {

  /*
   * Le D4 n'utilise pas de chauffage électrique.
   *
   * On conserve la fonction pour compatibilité avec
   * l'interface du composant mais on refuse volontairement
   * cette commande.
   */

  ESP_LOGW(
      TAG,
      "D4 2021: commande puissance electrique ignoree (%u W)",
      value
  );


  return false;
}



bool TrumaiNetBoxAppHeater::action_heater_energy_mix(
    EnergyMix energy_mix,
    ElectricPowerLevel el_power_level) {

  /*
   * Le D4 Diesel n'est pas un Combi E.
   *
   * Pour le moment nous ne permettons donc pas de modifier
   * manuellement le mix énergétique.
   */

  ESP_LOGW(
      TAG,
      "D4 2021: modification EnergyMix ignoree "
      "(mix=%u power=%u)",

      (uint16_t)energy_mix,
      (uint16_t)el_power_level
  );


  return false;
}


}  // namespace truma_inetbox
}  // namespace esphome

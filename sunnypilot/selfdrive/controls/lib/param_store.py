"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import capnp

from cereal import custom

from opendbc.car import structs
from openpilot.common.params import Params


class ParamStore:
  keys: list[str]
  values: dict[str, str]

  def __init__(self, CP: structs.CarParams):
    universal_params: list[str] = []
    brand_params: list[str] = []
    self.values = {}
    self.changed = False

    if CP.brand == "hyundai":
      brand_params.extend([
        "HyundaiLongitudinalTuning",
        "LongTuningCustomToggle",
        "LongTuningVEgoStopping",
        "LongTuningVEgoStarting",
        "LongTuningStoppingDecelRate",
        "LongTuningLongitudinalActuatorDelay",
        "LongTuningMinUpperJerk",
        "LongTuningMinLowerJerk",
        "LongTuningJerkLimits",
        "LongTuningLookaheadJerkBp",
        "LongTuningLookaheadJerkUpperV",
        "LongTuningLookaheadJerkLowerV",
        "LongTuningUpperJerkV",
        "LongTuningLowerJerkV",
      ])

    self.keys = universal_params + brand_params
    self.values = {}

  def update(self, params: Params) -> None:
    new_values = {k: params.get(k, encoding='utf8') or "0" for k in self.keys}
    self.changed = new_values != self.values
    self.values = new_values

  def publish(self) -> list[capnp.lib.capnp._DynamicStructBuilder]:
    if not self.changed:
      return []

    params_list: list[capnp.lib.capnp._DynamicStructBuilder] = []
    for k in self.keys:
      params_list.append(custom.CarControlSP.Param(key=k, value=self.values[k]))

    self.changed = False
    return params_list

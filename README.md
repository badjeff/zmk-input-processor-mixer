# Input Processor Mixer

This module interrupt, combine, sync incoming input events from Zephyr input subsystem for ZMK.

> [!IMPORTANT]
> This module is under development. Do NOT use in production. Breaking changes is unavoidable.

## What it does

To Be Determined

## Installation

Include this modulr on your ZMK's west manifest in `config/west.yml`:

```yaml
manifest:
  remotes:
    #...
    # START #####
    - name: badjeff
      url-base: https://github.com/badjeff
    # END #######
    #...
  projects:
    #...
    # START #####
    - name: zmk-input-processor-mixer
      remote: badjeff
      revision: main
    # END #######
    #...
```

Roughly, `overlay` of a 2-sensors trackball should look like below.

```
#include <input/processors/mixer.dtsi>

/{

  //** routing x-axis sensor `pd0` to `&zip_mixer`
  tball1_pri_mmv_il {
    compatible = "zmk,input-listener";
    device = <&pd0>;
    default {
      layers = <DEFAULT>;
      input-processors = <&zip_mixer 0>; // Sensor A (index 0)
    };
  };

  //** routing y-axis sensor `pd0` to `&zip_mixer`
  tball1_sec_mmv_il {
    compatible = "zmk,input-listener";
    device = <&pd0a>;
    default {
      layers = <DEFAULT>;
      input-processors = <&zip_mixer 1>; // Sensor B (index 1)
    };
  };

  //** routing mixer device as a valid output
  mixin_mmv_il {
    compatible = "zmk,input-listener";
    device = <&zip_mixer>;
  };

};
```

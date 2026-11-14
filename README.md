# esp-idf-lib/encoder

[![Build examples](https://github.com/esp-idf-lib/encoder/actions/workflows/build.yml/badge.svg)](https://github.com/esp-idf-lib/encoder/actions/workflows/build.yml)
[![Build docs](https://github.com/esp-idf-lib/encoder/actions/workflows/build-docs.yml/badge.svg)](https://github.com/esp-idf-lib/encoder/actions/workflows/build-docs.yml)
[![Validation](https://github.com/esp-idf-lib/encoder/actions/workflows/validate-component.yml/badge.svg)](https://github.com/esp-idf-lib/encoder/actions/workflows/validate-component.yml)

HW timer-based driver for incremental rotary encoders.

The encoder is polled by a periodic esp_timer, so the pins do not have to be
interrupt capable and several encoders work at the same time. The polling
interval is a build time constant, `CONFIG_RE_INTERVAL_US`, which also sets the
granularity of the button timings. The events are reported from the timer task,
so the callback has to be quick and must not call back into the driver; see the
documentation of the callback type before using it.

* [Documentation](https://esp-idf-lib.github.io/encoder/)
* [Repository](https://github.com/esp-idf-lib/encoder)
* [Issues](https://github.com/esp-idf-lib/encoder/issues)
* [Discussions and questions](https://github.com/esp-idf-lib/core/discussions)
* [Component page at the ESP Component Registry](https://components.espressif.com/components/esp-idf-lib/encoder)

## Installation

```sh
idf.py add-dependency esp-idf-lib/encoder
```

## Support

For questions and discussions about the component, please use
[Discussions](https://github.com/esp-idf-lib/core/discussions)
at [esp-idf-lib/core](https://github.com/esp-idf-lib/core).

## Contributing

Please read [CONTRIBUTING.md](https://github.com/esp-idf-lib/core/blob/main/CONTRIBUTING.md)
at [esp-idf-lib/core](https://github.com/esp-idf-lib/core).

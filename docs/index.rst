encoder: HW timer-based driver for incremental rotary encoders
==============================================================

The driver polls the pins of an incremental encoder with a periodic
``esp_timer``, so the pins do not have to be interrupt capable and any number
of encoders work at the same time. Every encoder has its own timer and its own
lock, so the encoders do not disturb each other.

The polling interval is not a part of the configuration: it is fixed at build
time by ``CONFIG_RE_INTERVAL_US``. The same value is the resolution of the
button timings, which are counted in polls, so a value that can be changed per
encoder would make a long press depend on the caller. One full detent of the
encoder takes four polls, and the button dead time, the long press threshold
and the acceleration use the same interval.

The events are reported from the timer task. The callback has to return
quickly, because every other timer of the system waits for it, and it must not
call back into the driver: ``rotary_encoder_delete()``,
``rotary_encoder_enable_acceleration()`` and
``rotary_encoder_disable_acceleration()`` refuse to run inside the callback
with ``ESP_ERR_INVALID_STATE``. Pass the events to another task through a
queue or a task notification instead.

With ESP-IDF 6.1 and newer, ``rotary_encoder_delete()`` waits for a callback
that is already running before it frees the handle. Older versions have no way
to wait: the timer stops dispatching events and the lock orders the callbacks
that have started, but a callback that the timer task has dispatched and that
has not taken the lock yet can still run after the handle is freed, so on those
versions delete the encoder only when the events are no longer expected.

 .. doxygenfile:: encoder.h

## What the example does

* Prints received events from the encoder such as "Button pressed" and the
  position value in a loop.
* Turns the acceleration on with a single click and off with a long press.
* Reports the events that were lost because the event queue was full, which
  happens when the encoder is turned faster than the main loop consumes the
  events.

## Wiring

Connect the common pin of the encoder to ground. The GPIO numbers are set with
`idf.py menuconfig` in "Example configuration": the pins A and B have to differ,
the button is optional. The defaults are:

| Pin    | Default GPIO |
|--------|--------------|
| A      | 16           |
| B      | 17           |
| Button | 18           |

The driver reads the button as active low, so the other side of the button goes
to ground. The encoder pins are the inputs, so enable the internal pull-ups of
the chip, or add external ones: a worn encoder bounces and rings, and without a
pull-up the noise reaches the counter.

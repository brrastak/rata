
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;


pub struct ThreeButtons<'a, IN1, IN2, IN3, OUT>
where
    IN1: InputPin,
    IN2: InputPin,
    IN3: InputPin,
    OUT: OutputPin
{
    input: (&'a IN1, &'a IN2, &'a IN3),
    output: &'a mut OUT
}

impl<'a, IN1, IN2, IN3, OUT> ThreeButtons<'a, IN1, IN2, IN3, OUT>
where
    IN1: InputPin,
    IN2: InputPin,
    IN3: InputPin,
    OUT: OutputPin
{
    pub fn new(input: (&'a IN1, &'a IN2, &'a IN3), output: &'a mut OUT) -> Self {

        output.set_high().ok();
        ThreeButtons { input, output }
    }

    pub fn prepare(&mut self) {

        self.output.set_low().ok();
    }

    pub fn is_pressed(&mut self) -> (bool, bool, bool) {

        let res = (self.input.0.is_low().ok().unwrap(), 
            self.input.1.is_low().ok().unwrap(), self.input.2.is_low().ok().unwrap());
        self.output.set_high().ok();

        res
    }
}

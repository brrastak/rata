
use embedded_hal::blocking::i2c::Write;
use pcf857x::Pcf8574;

pub struct VfdControl {
    digit: [u16; 5],
}

pub struct VfdDriver<'a, I2C>
where
    I2C: Write,
{
    high: &'a mut Pcf8574<I2C>,
    low: &'a mut Pcf8574<I2C>,
    current_pos: usize,
}

pub enum Error {
    InvalidInputData,
}

macro_rules! swap {
    ($x: expr) => {
        (($x & 0x0f) << 4) | (($x & 0xf0) >> 4)
    };
}
macro_rules! high {
    ($x: expr) => {
       swap!((1 << $x)) << 8
    };
}
macro_rules! low {
    ($x: expr) => {
        swap!(1 << ($x - 6))
    };
}

const SEG_A: u16 = low!(13);
const SEG_B: u16 = low!(10);
const SEG_C: u16 = high!(5);
const SEG_D: u16 = low!(6);
const SEG_E: u16 = high!(3);
const SEG_F: u16 = low!(11);
const SEG_G: u16 = high!(2);
const SEGMENTS: [u16; 10] =
    [SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
    SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
    SEG_B | SEG_C | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G, 
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, 
    SEG_A | SEG_B | SEG_C, 
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, 
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G];
const DP: u16 = low!(8);
const BELL: u16 = high!(0);

const POSITIONS: [u16; 5] =
    [high!(1), 
    high!(4), 
    low!(7), 
    low!(9), 
    low!(12)];
const DP_BELL_POS: usize = 2;
const NUMBER_OF_POS: usize = 5;

impl<'a, I2C, E> VfdDriver<'a, I2C>
where
    I2C: Write<Error = E>,
{
    pub fn new(high:&'a mut Pcf8574<I2C>, low:&'a mut Pcf8574<I2C>) -> Self {

        VfdDriver {
            high,
            low,
            current_pos: 0,
        }
    }

    pub fn update(&mut self, control: &VfdControl) -> Result<(), pcf857x::Error<E>> {

        let bits = control.digit[self.current_pos] | POSITIONS[self.current_pos];

        self.high.set((bits >> 8) as u8)?;
        self.low.set(bits as u8)?;

        self.current_pos += 1;
        self.current_pos %= NUMBER_OF_POS;

        Ok(())
    }
}

impl VfdControl
{
    pub fn new() -> Self {

        VfdControl {
            digit: [0; 5],
        }
    }

    pub fn set_dp_state(&mut self, new_state: bool) {

        if new_state == true {

            self.digit[DP_BELL_POS] |= DP;
        }
        else {
            self.digit[DP_BELL_POS] &= !DP;
        }
    }

    pub fn toggle_dp(&mut self) {

        self.digit[DP_BELL_POS] ^= DP;
    }


    pub fn set_bell_state(&mut self, new_state: bool) {
        
        if new_state == true {

            self.digit[DP_BELL_POS] |= BELL;
        }
        else {
            self.digit[DP_BELL_POS] &= !BELL;
        }
    }

    pub fn set_digit(&mut self, digit: u8, position: usize) -> Result<(), Error> {

        if digit > 9 || position > 3 {

            return Err(Error::InvalidInputData);
        }
        
        let index = if position < DP_BELL_POS { position } else { position + 1 };

        self.digit[index] = SEGMENTS[digit as usize];

        Ok(())
    }

    pub fn set_2digits(&mut self, number: u8, position: usize) -> Result<(), Error> {
        
        let one = number % 10;
        let ten = number / 10;
        let position = position * 2;

        self.set_digit(ten, position)?;
        self.set_digit(one, position + 1)?;

        Ok(())
    }

    pub fn set_4digits(&mut self, number: u16) -> Result<(), Error> {

        let one = (number % 100) as u8;
        let hundred = (number / 100) as u8;

        self.set_2digits(hundred, 0)?;
        self.set_2digits(one, 1)?;

        Ok(())      
    }

}
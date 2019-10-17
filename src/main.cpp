#include <array>
#include <tuple>
#include <LibreUCpp/HAL/Clock.h>
#include <LibreUCpp/HAL/ClockConfig.h>
#include <LibreUCpp/HAL/Port.h>
#include <LibreUCpp/HAL/Pin.h>
#include <LibreUCpp/HAL/UART.h>
#include <LibreUCpp/HAL/ADC.h>
#include <LibreUCpp/HAL/Systick.h>
#include "Midi.h"
#include "DrumStateMachine.h"

using namespace LibreUCpp::HAL;

volatile uint32_t TIME = 0;

extern "C" {
    void SysTick_Handler()
    {
        TIME++;
    }
}

Clock clk;
UART uart;
ADC adc;

void SetupAdc()
{
    Pin gnd { Port::A[3] };
    gnd.SetDirection(Pin::Direction::OUTPUT);
    gnd.Write(false);

    Pin ain0 { Port::A[2] };
    ain0.ConfigureMultiplex(Pin::Mux::B);

    Pin ain2 { Port::B[8] };
    ain2.ConfigureMultiplex(Pin::Mux::B);

    Pin ain3 { Port::B[9] };
    ain3.ConfigureMultiplex(Pin::Mux::B);

    adc.Setup(
        ADC::INSTANCE::ADC0,
        GCLK::GENERATOR::Generator_0,
        ADC::PRESCALER::DIV_BY_8,
        ADC::REFSEL::INTVCC2,
        ADC::RESOLUTION::RESOLUTION_10_BIT,
        ADC::DIFFMODE::SINGLE_ENDED,
        ADC::ACCUMULATION::ACCUMULATE_1_SAMPLE
    );
}

void SetupUart()
{
    Pin txd { Port::B[2] };
    txd.ConfigureMultiplex(Pin::Mux::D);

    Pin rxd { Port::B[3] };
    rxd.ConfigureMultiplex(Pin::Mux::D);


    uart.Setup(
        UART::INSTANCE::SERCOM5, 
        GCLK::GENERATOR::Generator_0,
        UART::RXPAD::RXPO1, UART::TXPAD::TXPO0,
        GCLK::CalcFrequency(GCLK::GENERATOR::Generator_0, clk.GetXosc32kFrequency(), clk.GetXoscFrequency())
    );
    uart.Init(115200, UART::BITS::EIGHT, UART::PARITY::NONE, UART::STOP_BITS::ONE);
}


int main()
{
    clk.Configure(ClockConfig::Get48MHzFrom16MHzCrystal());

    Systick::SetFrequency(clk, 1000);
    Systick::EnableInterrupt();
    Systick::Enable();

    SetupAdc();
    SetupUart();

    Midi midi(uart);

    using DrumTuple = std::tuple<ADC::CHANNEL, DrumStateMachine, Midi::PITCH>;
    std::array<DrumTuple, 3> drumTuples {
        DrumTuple { ADC::CHANNEL::AIN0, DrumStateMachine {}, Midi::PITCH::SNARE_1 },
        DrumTuple { ADC::CHANNEL::AIN2, DrumStateMachine {}, Midi::PITCH::MID_TOM_1 },
        DrumTuple { ADC::CHANNEL::AIN3, DrumStateMachine {}, Midi::PITCH::HIHAT_CLOSED },
    };

    while (true)
    {
        for (auto& [adcChannel, drumSM, pitch]: drumTuples)
        {
            if (drumSM.Update(TIME, adc.ReadChannel(adcChannel)))
            {
                if (drumSM.GetState() == DrumStateMachine::State::TRIGGERED)
                {
                    midi.SendNoteOn(Midi::CHANNEL_DRUMS, pitch, drumSM.GetVelocity());
                }
                else
                {
                    midi.SendNoteOff(Midi::CHANNEL_DRUMS, pitch, 0);
                }
            }
        }
    }
}

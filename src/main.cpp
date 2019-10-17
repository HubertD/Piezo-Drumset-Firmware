#include <array>
#include <numeric>
#include <tuple>
#include <LibreUCpp/HAL/Clock.h>
#include <LibreUCpp/HAL/ClockConfig.h>
#include <LibreUCpp/HAL/Port.h>
#include <LibreUCpp/HAL/Pin.h>
#include <LibreUCpp/HAL/UART.h>
#include <LibreUCpp/HAL/ADC.h>
#include <LibreUCpp/HAL/Systick.h>

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

class DrumStateMachine
{
    public:
        enum class State
        {
            IDLE,
            TRIGGERED,
        };

    public:
        DrumStateMachine() 
        {
            _buffer.fill(0);
        }

        bool Update(unsigned time, unsigned adcValue)
        {
            _buffer[_bufferPos++] = adcValue;
            _bufferPos %= BUFFER_SIZE;

            switch (_state)
            {
                case State::IDLE:
                    return UpdateStateIdle(time, adcValue);

                case State::TRIGGERED:
                    return UpdateStateTriggered(time, adcValue);

                default:
                    return false;
            }
        }

        State GetState()
        {
            return _state;
        }

        unsigned GetVelocity()
        {
            return _velocity;
        }

    private:
        static constexpr unsigned BUFFER_SIZE { 32 };
        static constexpr unsigned THRESHOLD_TRIGGER { 4000 };
        static constexpr unsigned THRESHOLD_OFF { 50 };
        static constexpr unsigned TIME_REFRACT_MS { 25 };
        static constexpr unsigned VELOCITY_DIVIDER { 128 };
        static constexpr unsigned MAX_VELOCITY { 127 };
        
        State _state;
        std::array<uint16_t, BUFFER_SIZE> _buffer;
        unsigned _bufferPos { 0 };
        unsigned _timeout { 0 };
        unsigned _velocity { 0 };

        bool UpdateStateIdle(unsigned time, unsigned)
        {
            auto sum = static_cast<unsigned>(std::accumulate(_buffer.begin(), _buffer.end(), 0));
            if (sum > THRESHOLD_TRIGGER)
            {
                _state = State::TRIGGERED;
                _timeout = time + TIME_REFRACT_MS;
                _velocity = sum / VELOCITY_DIVIDER;
                if (_velocity > MAX_VELOCITY)
                {
                    _velocity = MAX_VELOCITY;
                }
                return true;
            }
            return false;
        }

        bool UpdateStateTriggered(unsigned time, unsigned adcValue)
        {
            if (adcValue > THRESHOLD_OFF)
            {
                _timeout = time + TIME_REFRACT_MS;
            }
            else if (_timeout <= time)
            {
                _state = State::IDLE;
                return true;
            }
            return false;
        }
        
};

class MIDI
{
    public:
        enum class PITCH : uint8_t
        {
            SNARE_1 = 38,
            SNARE_2 = 40,
            HIHAT_CLOSED = 42,
            MID_TOM_1 = 47,
        };

        static constexpr uint8_t CHANNEL_DRUMS = 0x0A;

    public:
        MIDI(UART& uart)
            : _uart(uart)
        {
        }

        void SendNoteOn(uint8_t channel, PITCH pitch, uint8_t velocity)
        {
            _uart.WriteChar(STATUS_NOTE_ON | (channel & 0x0F));
            _uart.WriteChar(static_cast<uint8_t>(pitch));
            _uart.WriteChar(velocity & 0x7F);
        }

        void SendNoteOff(uint8_t channel, PITCH pitch, uint8_t velocity)
        {
            _uart.WriteChar(STATUS_NOTE_OFF | (channel & 0x0F));
            _uart.WriteChar(static_cast<uint8_t>(pitch));
            _uart.WriteChar(velocity & 0x7F);
        }

    private:
        static constexpr uint8_t STATUS_NOTE_ON = 0x90;
        static constexpr uint8_t STATUS_NOTE_OFF = 0x80;

        UART& _uart;
};


int main()
{
    clk.Configure(ClockConfig::Get48MHzFrom16MHzCrystal());

    Systick::SetFrequency(clk, 1000);
    Systick::EnableInterrupt();
    Systick::Enable();

    SetupAdc();
    SetupUart();

    MIDI midi(uart);

    using DrumTuple = std::tuple<ADC::CHANNEL, DrumStateMachine, MIDI::PITCH>;
    std::array<DrumTuple, 3> drumTuples {
        DrumTuple { ADC::CHANNEL::AIN0, DrumStateMachine {}, MIDI::PITCH::SNARE_1 },
        DrumTuple { ADC::CHANNEL::AIN2, DrumStateMachine {}, MIDI::PITCH::MID_TOM_1 },
        DrumTuple { ADC::CHANNEL::AIN3, DrumStateMachine {}, MIDI::PITCH::HIHAT_CLOSED },
    };

    while (true)
    {
        for (auto& tuple: drumTuples)
        {
            auto& adcChannel = std::get<0>(tuple);
            auto& drumSM = std::get<1>(tuple);
            auto& pitch = std::get<2>(tuple);

            if (drumSM.Update(TIME, adc.ReadChannel(adcChannel)))
            {
                if (drumSM.GetState() == DrumStateMachine::State::TRIGGERED)
                {
                    midi.SendNoteOn(MIDI::CHANNEL_DRUMS, pitch, drumSM.GetVelocity());
                }
                else
                {
                    midi.SendNoteOff(MIDI::CHANNEL_DRUMS, pitch, 0);
                }
            }
        }
    }
}

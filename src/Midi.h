#pragma once
#include <stdint.h>

template <class TX_CHANNEL_T> class Midi
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
        Midi(TX_CHANNEL_T& txChannel)
            : _txChannel(txChannel)
        {
        }

        void SendNoteOn(uint8_t channel, PITCH pitch, uint8_t velocity)
        {
            _txChannel.WriteChar(STATUS_NOTE_ON | (channel & 0x0F));
            _txChannel.WriteChar(static_cast<uint8_t>(pitch));
            _txChannel.WriteChar(velocity & 0x7F);
        }

        void SendNoteOff(uint8_t channel, PITCH pitch, uint8_t velocity)
        {
            _txChannel.WriteChar(STATUS_NOTE_OFF | (channel & 0x0F));
            _txChannel.WriteChar(static_cast<uint8_t>(pitch));
            _txChannel.WriteChar(velocity & 0x7F);
        }

    private:
        static constexpr uint8_t STATUS_NOTE_ON = 0x90;
        static constexpr uint8_t STATUS_NOTE_OFF = 0x80;

        TX_CHANNEL_T& _txChannel;
};

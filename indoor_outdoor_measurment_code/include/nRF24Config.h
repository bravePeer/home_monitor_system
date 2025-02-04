#pragma once
// extern "C"
// {
//   #include "avr/io.h"
//   #include "util/delay.h"
// }

#include <stdint.h>

//LSB first
enum class Commands: uint8_t {
    ReadRegister = 0x00,   //use address, 000A AAAA, 1-5 bytes 
    WriteRegister = 0x20,  //use address, 001A AAAA
    ReadRxPayload = 0b01100001, // 1-32 bytes data
    WriteTxPayload = 0b10100000, // 1-32 bytes data
    FlushTx = 0b11100001, // 0 bytes data
    FlushRx = 0b11100010, // 0 bytes data
    ReuseTxPayload = 0b11100011, // 0 bytes data
    ReadRxPayloadWidth = 0b01100000, // 1 bytes data
    WriteAckPayload = 0b10101000, // 10101PPP, 1-32 bytes data
    WriteTxPayloadNoAck = 0b10110000, // 1-32 bytes data
    Nop = 0b11111111
};

//Register map
enum class RegMap: uint8_t {
    Config = 0x00, // Configuration Register

    EnAA = 0x01,
    EnRxAddr = 0x02,
    SetupAw = 0x03,
    SetupRetr = 0x04,
    RFChannel = 0x05, // RF Channel
    RFSetup = 0x06, // RF Setup register
    Status = 0x07,
    ObserveTx = 0x08,
    Rpd = 0x09,

    RXAddressP0 = 0x0a, // Receive address data pipe
    RXAddressP1 = 0x0b, // Receive address data pipe
    RXAddressP2 = 0x0c, // Receive address data pipe
    RXAddressP3 = 0x0d, // Receive address data pipe
    RXAddressP4 = 0x0e, // Receive address data pipe
    RXAddressP5 = 0x0f, // Receive address data pipe

    TXAddress = 0x10, // Transmit address
    RxPwP0 = 0x11,
    RxPwP1 = 0x12,
    RxPwP2 = 0x13,
    RxPwP3 = 0x14,
    RxPwP4 = 0x15,
    RxPwP5 = 0x16,
    FifoStatus = 0x17,
    DynPd = 0x1c, // Enable dynamic payload lenght
    Feuture = 0x1d // Feuture Register
};

namespace Reg
{
    enum class Config: uint8_t {
        prim_rx = 0,
        pwr_up = 1,
        crc0 = 2,
        en_crc = 3,
        mask_max_rt = 4,
        mask_tx_ds = 5,
        mask_rx_dr = 6
    };

    enum class EnAA: uint8_t {
        enaa_p5 = 5,
        enaa_p4 = 4,
        enaa_p3 = 3,
        enaa_p2 = 2,
        enaa_p1 = 1,
        enaa_p0 = 0
    };

    enum class EnRxAddr: uint8_t {
        erx_p5 = 5,
        erx_p4 = 4,
        erx_p3 = 3,
        erx_p2 = 2,
        erx_p1 = 1,
        erx_p0 = 0
    };

    enum class SetupAW: uint8_t {
        aw1 = 1,
        aw0 = 0
    };

    enum class SetupRetr: uint8_t {
        ard7 = 7,
        ard6 = 6,
        ard5 = 5,
        ard4 = 4,
        arc3 = 3,
        arc2 = 2,
        arc1 = 1,
        arc0 = 0,
    };

    enum class RfCh: uint8_t {
        rf_ch7 = 7,
        rf_ch6 = 6,
        rf_ch5 = 5,
        rf_ch4 = 4,
        rf_ch3 = 3,
        rf_ch2 = 2,
        rf_ch1 = 1,
        rf_ch0 = 0,
    };

    enum class RfSetup: uint8_t {
        cont_wave = 7,
        rf_dr_low = 5,
        pll_lock = 4,
        rf_dr_high = 3,
        rf_pwr2 = 2,
        rf_pwr1 = 1
    };

    enum class Status: uint8_t {
        rx_dr = 6, // Data Ready IRQ
        tx_ds = 5, // Data Send IRQ
        max_rt = 4, // Max Retransmissions IRQ
        rx_p_no3 = 3,
        rx_p_no2 = 2,
        rx_p_no1 = 1,
        tx_full = 0
    };

    enum class ObserveTx: uint8_t {
        plos_cnt7 = 7,
        plos_cnt6 = 6,
        plos_cnt5 = 5,
        plos_cnt4 = 4,
        arc_cnt3 = 3,
        arc_cnt2 = 2,
        arc_cnt1 = 1,
        arc_cnt0 = 0
    };

    enum class RPD: uint8_t {
        rpd = 0
    };

    enum class RxPwP0: uint8_t {
        rx_pw_p5 = 5,
        rx_pw_p4 = 4,
        rx_pw_p3 = 3,
        rx_pw_p2 = 2,
        rx_pw_p1 = 1,
        rx_pw_p0 = 0
    };

    enum class RxPwP1: uint8_t {
        rx_pw_p5 = 5,
        rx_pw_p4 = 4,
        rx_pw_p3 = 3,
        rx_pw_p2 = 2,
        rx_pw_p1 = 1,
        rx_pw_p0 = 0
    };

    enum class RxPwP2: uint8_t {
        rx_pw_p5 = 5,
        rx_pw_p4 = 4,
        rx_pw_p3 = 3,
        rx_pw_p2 = 2,
        rx_pw_p1 = 1,
        rx_pw_p0 = 0
    };

    enum class RxPwP3: uint8_t {
        rx_pw_p5 = 5,
        rx_pw_p4 = 4,
        rx_pw_p3 = 3,
        rx_pw_p2 = 2,
        rx_pw_p1 = 1,
        rx_pw_p0 = 0
    };

    enum class RxPwP4: uint8_t {
        rx_pw_p5 = 5,
        rx_pw_p4 = 4,
        rx_pw_p3 = 3,
        rx_pw_p2 = 2,
        rx_pw_p1 = 1,
        rx_pw_p0 = 0
    };

    enum class RxPwP5: uint8_t {
        rx_pw_p5 = 5,
        rx_pw_p4 = 4,
        rx_pw_p3 = 3,
        rx_pw_p2 = 2,
        rx_pw_p1 = 1,
        rx_pw_p0 = 0
    };

    enum class FifoStatus: uint8_t {
        tx_reuse = 6,
        tx_full = 5,
        tx_empty = 4,
        rx_full = 1,
        rx_empty = 0
    };

    enum class Dynpd: uint8_t {
        dpl_p5 = 5,
        dpl_p4 = 4,
        dpl_p3 = 3,
        dpl_p2 = 2,
        dpl_p1 = 1,
        dpl_p0 = 0,
    };

    enum class Feature: uint8_t {
        en_dpl = 2,
        en_ack_pay = 1,
        en_dyn_ack = 0
    };
}

constexpr uint8_t WriteRegister(RegMap reg)
{
    return static_cast<uint8_t>(Commands::WriteRegister) | static_cast<uint8_t>(reg);
}

constexpr uint8_t ReadRegister(RegMap reg)
{
    return static_cast<uint8_t>(Commands::ReadRegister) | static_cast<uint8_t>(reg);
}

constexpr uint8_t WriteCmd(Commands cmd)
{
    return static_cast<uint8_t>(cmd);
}

constexpr uint8_t Cmd(Commands cmd)
{
    return static_cast<uint8_t>(cmd);
}

constexpr uint8_t SetRegister() 
{
    return 0;
}

template<typename T, typename... Args>
constexpr uint8_t SetRegister(const T& first, const Args&... rest) 
{
    return (1<<static_cast<uint8_t>(first)) | SetRegister(rest...);
}

#[repr(u8)]
/// Serial command opcodes
pub enum Command {
    NoOp = 0x00,
    TriggerSet = 0x01,
    AvgNumSet = 0x02,
    TriggerWindowSet = 0x03,
    TriggerIntervalSet = 0x04,
    TriggerSingleSet = 0x05,
    AverageStart = 0x06,
    AverageStop = 0x07,
    RangeSet = 0x08,
    LcdSet = 0x09,
    TriggerStop = 0x0A,
    DeviceRunningSet = 0x0C,
    RegulatorSet = 0x0D,
    SwitchPointDown = 0x0E,
    SwitchPointUp = 0x0F,
    TriggerExtToggle = 0x10,
    SetPowerMode = 0x11,
    ResUserSet = 0x12,
    SpikeFilteringOn = 0x15,
    SpikeFilteringOff = 0x16,
    GetMetaData = 0x19,
    Reset = 0x20,
    SetUserGains = 0x25,
}
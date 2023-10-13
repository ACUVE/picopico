use core::num::NonZeroU32;

pub(crate) const READ: u8 = 0x28;
pub(crate) const READ_CAPACITY: u8 = 0x25;
pub(crate) const START_STOP_UNIT: u8 = 0x1b;
pub(crate) const SYNCHRONIZE_CACHE: u8 = 0x35;
pub(crate) const WRITE: u8 = 0x2a;
pub(crate) const VERIFY: u8 = 0x2f;
pub(crate) const INQUIRY: u8 = 0x12;
pub(crate) const MODE_SELECT: u8 = 0x55;
pub(crate) const MODE_SENSE: u8 = 0x5a;
pub(crate) const TEST_UNIT_READY: u8 = 0x00;
pub(crate) const WRITE_BUFFER: u8 = 0x3b;

pub(crate) struct RbcHandler {}

// 真面目に実装する気はないので、送受信か否かと送受信するバイト数だけ返す
pub(crate) enum Data<'a> {
    None,
    SendDummy(NonZeroU32),
    RecvDummy(NonZeroU32),
    Send(&'a [u8]),
}

impl RbcHandler {
    pub(crate) fn new() -> Self {
        Self {}
    }

    pub(crate) fn handle<'a>(
        &mut self,
        data: &[u8],
        buffer: &'a mut [u8; 32],
    ) -> Result<Option<Data<'a>>, ()> {
        if data.len() < 1 {
            return Err(());
        }

        Ok(match data[0] {
            TEST_UNIT_READY => Some(Data::None),
            READ_CAPACITY => {
                parse_read_capacity(data)?;
                buffer[0..4].copy_from_slice(&0x00000000u32.to_le_bytes());
                buffer[4..8].copy_from_slice(&0xFFFFFFFFu32.to_le_bytes());
                Some(Data::Send(&buffer[0..8]))
            }
            READ => {
                let (_address, length) = parse_read(data)?;
                if length == 0 {
                    Some(Data::None)
                } else {
                    Some(Data::SendDummy(NonZeroU32::new(length as u32).unwrap()))
                }
            }
            WRITE => {
                let (_address, length) = parse_write(data)?;
                if length == 0 {
                    Some(Data::None)
                } else {
                    Some(Data::RecvDummy(NonZeroU32::new(length as u32).unwrap()))
                }
            }
            INQUIRY => {
                const INQUIRY_DATA: [u8; 36] = [
                    0x0E, 0x80, 0x04, 0x02, 27, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ];
                let (evpd, page_code, allocation_length) = parse_inquiry(data)?;
                if allocation_length == 0 {
                    Some(Data::None)
                } else {
                    if evpd == false {
                        if page_code != 0x00 {
                            return Err(());
                        }
                        Some(Data::Send(&INQUIRY_DATA[..allocation_length as usize]))
                    } else {
                        // Some(Data::SendDummy(
                        //     NonZeroU32::new(allocation_length as u32).unwrap(),
                        // ))
                        return Err(());
                    }
                }
            }
            START_STOP_UNIT | SYNCHRONIZE_CACHE | VERIFY | MODE_SELECT | MODE_SENSE
            | WRITE_BUFFER => Some(Data::None),
            _ => return Err(()),
        })
    }
}

pub(crate) fn parse_read(data: &[u8]) -> Result<(u32, u16), ()> {
    if data[0] != READ {
        return Err(());
    }
    if data.len() != 10 {
        return Err(());
    }
    if data[9] != 0 {
        return Err(());
    }
    let address = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
    let length = u16::from_le_bytes([data[7], data[8]]);
    Ok((address, length))
}

pub(crate) fn parse_read_capacity(data: &[u8]) -> Result<(), ()> {
    if data[0] != READ_CAPACITY {
        return Err(());
    }
    if data.len() != 10 {
        return Err(());
    }
    if data[9] != 0 {
        return Err(());
    }
    Ok(())
}

pub(crate) fn parse_synchronize_cache(data: &[u8]) -> Result<(), ()> {
    if data[0] != SYNCHRONIZE_CACHE {
        return Err(());
    }
    if data.len() != 10 {
        return Err(());
    }
    if data[9] != 0 {
        return Err(());
    }
    Ok(())
}

pub(crate) fn parse_write(data: &[u8]) -> Result<(u32, u16), ()> {
    if data[0] != WRITE {
        return Err(());
    }
    if data.len() != 10 {
        return Err(());
    }
    if data[9] != 0 {
        return Err(());
    }
    let address = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
    let length = u16::from_le_bytes([data[7], data[8]]);
    Ok((address, length))
}

pub(crate) fn parse_verify(data: &[u8]) -> Result<(u32, u16), ()> {
    if data[0] != VERIFY {
        return Err(());
    }
    if data.len() != 10 {
        return Err(());
    }
    if data[9] != 0 {
        return Err(());
    }
    let address = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
    let length = u16::from_le_bytes([data[7], data[8]]);
    Ok((address, length))
}

pub(crate) fn parse_inquiry(data: &[u8]) -> Result<(bool, u8, u16), ()> {
    if data[0] != INQUIRY {
        return Err(());
    }
    if data.len() != 6 {
        return Err(());
    }
    if data[5] != 0 {
        return Err(());
    }
    let evpd = data[1] & 0x01 != 0;
    let cmd_dt = data[1] & 0x02 != 0;
    let page_code = data[2];
    let allocation_length = u16::from_le_bytes([data[3], data[4]]);
    Ok((evpd, page_code, allocation_length))
}

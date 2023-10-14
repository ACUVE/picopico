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
        buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        if data.len() < 1 {
            return Err(());
        }

        Ok(match data[0] {
            TEST_UNIT_READY => self.handle_test_unit_ready(data, buffer)?,
            READ_CAPACITY => self.handle_read_capacity(data, buffer)?,
            READ => self.handle_read(data, buffer)?,
            WRITE => self.handle_write(data, buffer)?,
            INQUIRY => self.handle_inquiry(data, buffer)?,
            START_STOP_UNIT | SYNCHRONIZE_CACHE | VERIFY | MODE_SELECT | MODE_SENSE
            | WRITE_BUFFER => Err(())?,
            _ => return Err(()),
        })
    }

    fn handle_test_unit_ready<'a>(
        &mut self,
        data: &[u8],
        _buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        parse_test_unit_ready(data)?;
        Ok(Some(Data::None))
    }

    fn handle_read_capacity<'a>(
        &mut self,
        data: &[u8],
        buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        parse_read_capacity(data)?;
        buffer[0..4].copy_from_slice(&0x00000000u32.to_le_bytes());
        buffer[4..8].copy_from_slice(&0xFFFFFFFFu32.to_le_bytes());
        Ok(Some(Data::Send(&buffer[0..8])))
    }

    fn handle_read<'a>(
        &mut self,
        data: &[u8],
        _buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        let (address, length) = parse_read(data)?;
        if length == 0 {
            Ok(Some(Data::None))
        } else {
            Ok(Some(Data::SendDummy(
                NonZeroU32::new(length as u32).unwrap(),
            )))
        }
    }

    fn handle_write<'a>(
        &mut self,
        data: &[u8],
        _buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        let (address, length) = parse_write(data)?;
        if length == 0 {
            Ok(Some(Data::None))
        } else {
            Ok(Some(Data::RecvDummy(
                NonZeroU32::new(length as u32).unwrap(),
            )))
        }
    }

    fn handle_inquiry<'a>(
        &mut self,
        data: &[u8],
        buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        const INQUIRY_DATA: [u8; 36] = [
            0x0E, // Peripheral Qualifier, Peripheral Device Type
            0x00, // RMB, Reserved
            0x00, // Version
            0x00, // AERC, Reserved, NormACA, HiSup, Response Data Format
            31,   // Additional Length
            0x00, //SCCS, Reserved
            0x00, // BQue, EncServ, VS, MultiP, MChngr, Addr16
            0x00, // RelAdr, WBus16, Sync, Linked, CmdQue, VS
            0, 0, 0, 0, 0, 0, 0, 0, // Vendor Identification
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Product Identification
            0, 0, 0, 0, // Product Revision Level
        ];
        let (evpd, cmdt, page_or_operation_code, allocation_length) = parse_inquiry(data)?;
        if allocation_length == 0 {
            Ok(Some(Data::None))
        } else {
            if evpd == false {
                match page_or_operation_code {
                    0x00 => Ok(Some(Data::Send(&INQUIRY_DATA[..allocation_length as _]))),
                    _ => Err(()),
                }
            } else {
                if cmdt == false {
                    match page_or_operation_code {
                        0x83 => {
                            // Device identification page
                            buffer[0..2].copy_from_slice(&INQUIRY_DATA[0..2]);
                            buffer[3] = 0x83;
                            buffer[4] = 0;
                            Ok(Some(Data::Send(&buffer[0..5][..allocation_length as _])))
                        }
                        0x80 => {
                            // Unit serial number page
                            buffer[0..2].copy_from_slice(&INQUIRY_DATA[0..2]);
                            buffer[3] = 0x80;
                            buffer[4] = 1;
                            buffer[5] = b' ';
                            Ok(Some(Data::Send(&buffer[0..6][..allocation_length as _])))
                        }
                        _ => Ok(None),
                    }
                } else {
                    Err(())
                }
            }
        }
    }
}

fn parse_test_unit_ready(data: &[u8]) -> Result<(), ()> {
    if data[0] != TEST_UNIT_READY {
        return Err(());
    }
    if data.len() != 6 {
        return Err(());
    }
    if data[5] != 0 {
        return Err(());
    }
    Ok(())
}

fn parse_read(data: &[u8]) -> Result<(u32, u16), ()> {
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

fn parse_read_capacity(data: &[u8]) -> Result<(), ()> {
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

fn parse_synchronize_cache(data: &[u8]) -> Result<(), ()> {
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

fn parse_write(data: &[u8]) -> Result<(u32, u16), ()> {
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

fn parse_verify(data: &[u8]) -> Result<(u32, u16), ()> {
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

fn parse_inquiry(data: &[u8]) -> Result<(bool, bool, u8, u8), ()> {
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
    let page_or_operation_code = data[2];
    let allocation_length = data[4];
    Ok((evpd, cmd_dt, page_or_operation_code, allocation_length))
}

use core::num::NonZeroU32;

pub(crate) const INQUIRY: u8 = 0x12;
pub(crate) const MODE_SELECT: u8 = 0x15;
pub(crate) const MODE_SENSE: u8 = 0x1a;
// pub(crate) const PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1e;
pub(crate) const READ: u8 = 0x28;
pub(crate) const READ_CAPACITY: u8 = 0x25;
pub(crate) const START_STOP_UNIT: u8 = 0x1b;
pub(crate) const TEST_UNIT_READY: u8 = 0x00;
pub(crate) const SYNCHRONIZE_CACHE: u8 = 0x35;
pub(crate) const VERIFY: u8 = 0x2f;
pub(crate) const WRITE: u8 = 0x2a;
pub(crate) const WRITE_BUFFER: u8 = 0x3b;

pub(crate) struct RbcHandler {
    logical_block_size: u16,
    // Page Code: 0x06
    write_cache_disable: bool,
    logical_block_count: u32,
}

// 真面目に実装する気はないので、送受信か否かと送受信するバイト数だけ返す
pub(crate) enum Data<'a> {
    None,
    SendDummy(NonZeroU32),
    RecvDummy(NonZeroU32),
    Send(&'a [u8]),
}

impl Default for RbcHandler {
    fn default() -> Self {
        Self {
            logical_block_size: 4096,
            write_cache_disable: true,
            logical_block_count: 0xFFFFFFFF,
        }
    }
}

impl RbcHandler {
    pub(crate) fn new() -> Self {
        Self {
            ..Default::default()
        }
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
            INQUIRY => self.handle_inquiry(data, buffer)?,
            // MODE_SELECT => self.handle_mode_select(data, buffer)?,
            MODE_SENSE => self.handle_mode_sense(data, buffer)?,
            TEST_UNIT_READY => self.handle_test_unit_ready(data, buffer)?,
            READ_CAPACITY => self.handle_read_capacity(data, buffer)?,
            READ => self.handle_read(data, buffer)?,
            WRITE => self.handle_write(data, buffer)?,
            START_STOP_UNIT | SYNCHRONIZE_CACHE | VERIFY | MODE_SELECT | MODE_SELECT
            | WRITE_BUFFER => Some(Data::None),
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
        buffer[0..=3].copy_from_slice(&self.logical_block_count.to_be_bytes());
        buffer[4..=7].copy_from_slice(&(self.logical_block_size as u32).to_be_bytes());
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
            0x1F, // Additional Length
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
                    0x00 => Ok(Some(Data::Send(
                        &INQUIRY_DATA
                            [..core::cmp::min(INQUIRY_DATA.len() as _, allocation_length) as _],
                    ))),
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
                            Ok(Some(Data::Send(
                                &buffer[..core::cmp::min(5, allocation_length) as _],
                            )))
                        }
                        0x80 => {
                            // Unit serial number page
                            buffer[0..2].copy_from_slice(&INQUIRY_DATA[0..2]);
                            buffer[3] = 0x80;
                            buffer[4] = 1;
                            buffer[5] = b' ';
                            Ok(Some(Data::Send(
                                &buffer[0..core::cmp::min(6, allocation_length) as _],
                            )))
                        }
                        _ => Ok(None),
                    }
                } else {
                    Err(())
                }
            }
        }
    }

    fn handle_mode_select<'a>(
        &mut self,
        data: &[u8],
        _buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        // TODO: Implement
        Ok(None)
    }

    fn handle_mode_sense<'a>(
        &mut self,
        data: &[u8],
        buffer: &'a mut [u8],
    ) -> Result<Option<Data<'a>>, ()> {
        let (dbd, pc, page_code, allocation_length) = parse_mode_sense(data)?;
        // dbd: Disable block descriptors
        // pc == 0: Current values, pc == 1: Changeable values
        // pc == 2: Default values, pc == 3: Saved values
        // page_code == 0x3f: All pages

        if !dbd {
            // RBC では dbd == true でなければならない
            return Err(());
        }
        if page_code != 0x06 {
            // RBC では page_code == 0x06 でなければならない
            return Err(());
        }
        // buffer[0] = 0x00; // Mode Data Length
        // buffer[1] = 0x0E; // Medium Type
        // buffer[2] = 0x00; // Device-Specific Parameter
        // buffer[3] = 0x00; // Block Descriptor Length
        // let len_06 = self.handle_mode_sense_06(&mut buffer[4..])?.0.len();

        // buffer[0] = len_06.saturating_add(3) as u8;

        Ok(Some(Data::Send(
            &buffer[..core::cmp::min(buffer[0].saturating_add(1), allocation_length) as _],
        )))
    }

    fn handle_mode_sense_06<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<(&'a mut [u8], &'a mut [u8]), ()> {
        if buffer.len() < 10 {
            return Err(());
        }
        buffer[0] = 0x86;
        buffer[1] = 0x08;
        buffer[2] = self.write_cache_disable as u8;
        buffer[3..=4].copy_from_slice(&self.logical_block_size.to_be_bytes());
        buffer[5] = 0x00;
        buffer[6..=9].copy_from_slice(&self.logical_block_count.to_be_bytes()); // 5 bytes!?
        Ok(buffer.split_at_mut(10))
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
    let address = u32::from_be_bytes([data[2], data[3], data[4], data[5]]);
    let length = u16::from_be_bytes([data[7], data[8]]);
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
    let address = u32::from_be_bytes([data[2], data[3], data[4], data[5]]);
    let length = u16::from_be_bytes([data[7], data[8]]);
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
    let address = u32::from_be_bytes([data[2], data[3], data[4], data[5]]);
    let length = u16::from_be_bytes([data[7], data[8]]);
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

fn parse_mode_sense(data: &[u8]) -> Result<(bool, u8, u8, u8), ()> {
    if data[0] != MODE_SENSE {
        return Err(());
    }
    if data.len() != 6 {
        return Err(());
    }
    if data[5] != 0 {
        return Err(());
    }
    let dbd = data[1] & 0x08 != 0;
    let pc = (data[2] & 0xc0) >> 6;
    let page_code = data[2] & 0x3f;
    let allocation_length = data[4];
    Ok((dbd, pc, page_code, allocation_length))
}

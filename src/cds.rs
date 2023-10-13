#[derive(Debug)]
pub(crate) struct Cbw<'a> {
    pub(crate) dCBWTag: u32,
    pub(crate) dCBWDataTransferLength: u32,
    pub(crate) bmCBWFlags: u8,
    pub(crate) bCBWLUN: u8,
    pub(crate) bCBWCBLength: u8,
    pub(crate) CBWCB: &'a [u8],
}

#[derive(Debug)]
pub(crate) enum ParseCbwError {
    InvalidSignature,
    BufferTooSmall,
    LengthZero,
    LengthOverflow,
}

pub(crate) fn parse_cbw(buff: &[u8]) -> Result<Cbw, ParseCbwError> {
    if buff.len() < 16 {
        return Err(ParseCbwError::BufferTooSmall);
    }
    let signature = u32::from_le_bytes([buff[0], buff[1], buff[2], buff[3]]);
    if signature != 0x43425355 {
        return Err(ParseCbwError::InvalidSignature);
    }
    let len = buff[14] & 0x1F;
    if len == 0 {
        return Err(ParseCbwError::LengthZero);
    }
    if len > 0x10 {
        return Err(ParseCbwError::LengthOverflow);
    }
    Ok(Cbw {
        dCBWTag: u32::from_le_bytes([buff[4], buff[5], buff[6], buff[7]]),
        dCBWDataTransferLength: u32::from_le_bytes([buff[8], buff[9], buff[10], buff[11]]),
        bmCBWFlags: buff[12] & 0x0F,
        bCBWLUN: buff[13] & 0x1F,
        bCBWCBLength: len,
        CBWCB: &buff[15..15 + len as usize],
    })
}

#[derive(Debug)]
pub(crate) struct Csw {
    pub(crate) dCSWTag: u32,
    pub(crate) dCSWDataResidue: u32,
    pub(crate) bCSWStatus: u8,
}

pub(crate) fn byteify_csw<'a, 'b>(
    buff: &'a mut [u8],
    csw: &'b Csw,
) -> Result<(&'a [u8], &'a mut [u8]), ()> {
    if buff.len() < 13 {
        return Err(());
    }
    buff[0..4].copy_from_slice(&0x53425355u32.to_le_bytes());
    buff[4..8].copy_from_slice(&csw.dCSWTag.to_le_bytes());
    buff[8..12].copy_from_slice(&csw.dCSWDataResidue.to_le_bytes());
    buff[12] = csw.bCSWStatus;
    let (send_buff, rest_buff) = buff.split_at_mut(13);
    Ok((send_buff, rest_buff))
}

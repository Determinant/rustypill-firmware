#[derive(Debug)]
pub enum GeneralError {
    BufferOverflowError,
}

#[macro_export]
macro_rules! write {
    ($dst:expr, $($arg:tt)*) => (core::write!($dst, $($arg)*).or(Err(crate::util::GeneralError::BufferOverflowError)))
}

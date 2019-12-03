use stm32f1xx_hal::stm32::Interrupt;
use stm32f1::stm32f103::NVIC;

pub struct MutexGuard<T>(*mut T, bool);

impl<T> Drop for MutexGuard<T> {
    fn drop(&mut self) {
        if self.1 {
            unsafe { NVIC::unmask(Interrupt::USART2); }
        }
    }
}

impl<T> core::ops::Deref for MutexGuard<T> {
    type Target = T;
    fn deref(&self) -> &T { unsafe {&*self.0} }
}

impl<T> core::ops::DerefMut for MutexGuard<T> {
    fn deref_mut(&mut self) -> &mut T { unsafe {&mut *self.0} }
}

pub struct Mutex<T> {
    data: core::cell::UnsafeCell<T>
}

impl<T> Mutex<T> {
    pub fn new(v: T) -> Self {
        Mutex {
            data: core::cell::UnsafeCell::new(v)
        }
    }

    pub fn lock(&self) -> MutexGuard<T> {
        let unmask = if NVIC::is_enabled(Interrupt::USART2) {
            NVIC::mask(Interrupt::USART2);
            true
        } else { false };
        MutexGuard(self.data.get(), unmask)
    }
}

unsafe impl<T> Sync for Mutex<T> {}


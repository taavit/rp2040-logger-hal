use core::str::FromStr;

use core::fmt::Write;
use embedded_sdmmc::{Directory, File, TimeSource, Timestamp, Volume, VolumeManager};
use heapless::String;

use crate::{uart_write, SdCardType, CACHE};

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub type VolumeManagerType = VolumeManager<SdCardType, DummyTimesource>;
pub struct SdWriter {
    file: Option<File>,
    volume_mgr: VolumeManagerType,
    volume: Volume,
    root_dir: Option<Directory>,
    file_idx: u16,
    base_filename: String<16>,
    filename: String<32>,
}

fn retry_open_volume(
    volume_mgr: &mut VolumeManagerType,
    times: usize,
) -> Result<Volume, embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
    let mut last: Option<Result<Volume, embedded_sdmmc::Error<embedded_sdmmc::SdCardError>>> = None;

    for _ in 0..times {
        let res = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0));
        if let Ok(res) = res {
            return Ok(res);
        }
        last = Some(res);
    }

    last.unwrap()
}

impl SdWriter {
    pub fn new(file: &str, mut volume_mgr: VolumeManagerType) -> Self {
        let volume = retry_open_volume(&mut volume_mgr, 5).unwrap();
        let filename = String::new();
        Self {
            volume,
            volume_mgr,
            file: None,
            file_idx: 1,
            base_filename: String::from_str(file).unwrap(),
            filename,
            root_dir: None,
        }
    }
    pub fn write(&mut self, data: &str) {
        critical_section::with(|cs| {
            if let Some(ref mut file) = self.file {
                let mut cache = CACHE.borrow(cs).borrow_mut();
                cache.push(String::from_str(data).unwrap()).unwrap();
                if cache.is_full() {
                    let mut full_string = String::<640>::new();
                    for e in cache.iter() {
                        full_string.push_str(e.as_str()).unwrap();
                    }
                    cache.clear();
                    for i in 0..5 {
                        let res = self.volume_mgr.write(*file, full_string.as_bytes());
                        if res.is_ok() {
                            break;
                        }
                        if i == 4 {
                            uart_write("Cannot write data to sd card\r\n");
                        }
                    }
                }
            }
        });
    }

    pub fn start_recording(&mut self) {
        self.generate_filename();

        critical_section::with(|cs| {
            CACHE.borrow(cs).borrow_mut().clear();
            if let Some(dir) = self.root_dir.take() {
                self.volume_mgr.close_dir(dir).unwrap();
            }
            let root_dir = self.volume_mgr.open_root_dir(self.volume).unwrap();
            let csv_file = self
                .volume_mgr
                .open_file_in_dir(
                    root_dir,
                    self.filename.as_str(),
                    embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
                )
                .unwrap();
            self.file = Some(csv_file);
            self.root_dir = Some(root_dir);
        });
    }

    pub fn stop_recording(&mut self) {
        critical_section::with(|cs| {
            if let Some(file) = self.file.take() {
                let mut cache = CACHE.borrow(cs).borrow_mut();

                let mut full_string = String::<640>::new();
                for e in cache.iter() {
                    full_string.push_str(e.as_str()).unwrap();
                }
                cache.clear();
                self.volume_mgr.write(file, full_string.as_bytes()).unwrap();
                self.volume_mgr.close_file(file).unwrap();
            };
            let root_dir = self.root_dir.take();
            if let Some(root_dir) = root_dir {
                self.volume_mgr.close_dir(root_dir).unwrap();
            }
            self.file_idx += 1;
        })
    }

    fn generate_filename(&mut self) {
        self.filename.clear();
        write!(
            self.filename,
            "{}_{:05}.csv",
            self.base_filename, self.file_idx
        )
        .unwrap();
    }
}

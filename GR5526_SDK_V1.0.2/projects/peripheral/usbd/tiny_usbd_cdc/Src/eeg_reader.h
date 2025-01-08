#ifndef EEG_READER_H_
#define EEG_READER_H_
#ifdef __cplusplus
extern "C" {
#endif

void eeg_reader_init(void);

void eeg_reader_task(void *arg);

#ifdef __cplusplus
}
#endif
#endif /* EEG_READER_H_ */

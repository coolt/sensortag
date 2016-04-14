#define BLE_ADV_PAYLOAD_BUF_LEN     64

void initRadio(void);
void runRadio(void);
void initRadioInts(void);
void initRadioStructs(void);

void radioPatch(void);
void radioCmdBusRequest(bool enabled);

void radioUpdateAdvData(int length,uint8_t* data);

void radioCmdStartRAT(void);
void radioSetupAndTransmit(void);

#include "stdint.h"
#include "v5_apitypes.h"

extern "C" {
    int32_t  vexAdiAddrLedSet( uint32_t index, uint32_t port, uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options );
    int32_t vexDeviceAdiAddrLedSet( V5_DeviceT device, uint32_t port, uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options );
}
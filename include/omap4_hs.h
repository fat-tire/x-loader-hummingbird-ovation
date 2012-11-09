#include <asm/arch/cpu.h>
#include <asm/io.h>

#define API_HAL_BASE_INDEX    0x00000000
#define NB_MAX_API_HAL    32
#define PPA_HAL_SERVICES_START_INDEX       (API_HAL_BASE_INDEX + NB_MAX_API_HAL)
#define PPA_SERV_HAL_CHICKENBITREG2           (PPA_HAL_SERVICES_START_INDEX + 7)
#define PPA_SERV_HAL_BN_INIT               (PPA_HAL_SERVICES_START_INDEX + 17)
#define PPA_SERV_HAL_BN_CHK              (PPA_HAL_SERVICES_START_INDEX + 18)
#define PPA_SERV_HAL_BN_GETMSV             (PPA_HAL_SERVICES_START_INDEX + 19)
void pmic_set_vpp(void);
void pmic_close_vpp(void);

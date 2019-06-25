#ifndef     USER_APP_LOG_H
#define     USER_APP_LOG_H

//#include    "SEGGER_RTT.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"

#if NRF_LOG_ENABLED

#define DBG_E(...)      NRF_LOG_ERROR(__VA_ARGS__);
#define DBG_W(...)      NRF_LOG_WARNING(__VA_ARGS__);
#define DBG_I(...)      NRF_LOG_INFO(__VA_ARGS__);
#define DBG_D(...)      NRF_LOG_DEBUG(__VA_ARGS__);
#define DBG_F(...)      NRF_LOG_FLUSH(__VA_ARGS__);

#define DBG_LOG(...)    NRF_LOG_INFO(__VA_ARGS__);
#define DBG_B(...)      NRF_LOG_DEBUG(__VA_ARGS__)

#else //NRF_LOG_DISABLED
#define DBG_E(...) 
#define DBG_W(...) 
#define DBG_I(...) 
#define DBG_D(...) 
#define DBG_F(...) 

#define DBG_LOG(...)     
#define DBG_B(...)      

#endif //NRF_LOG_ENABLED


#endif  //USER_APP_LOG_H


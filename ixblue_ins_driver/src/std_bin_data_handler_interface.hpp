//
// Created by ejalaa on 14/01/2022.
//

#ifndef IXBLUE_INS_DRIVER_STD_BIN_DATA_HANDLER_INTERFACE_HPP_
#define IXBLUE_INS_DRIVER_STD_BIN_DATA_HANDLER_INTERFACE_HPP_

#include <ixblue_stdbin_decoder/stdbin_decoder.h>

class StdBinDataHandlerInterface
{
public:
  virtual void onNewStdBinData(
    const ixblue_stdbin_decoder::Data::BinaryNav & navData,
    const ixblue_stdbin_decoder::Data::NavHeader & headerData) = 0;
};


#endif //IXBLUE_INS_DRIVER_STD_BIN_DATA_HANDLER_INTERFACE_HPP_

/*
 * @Description:debug小工具
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:
 * @Date: 2022-10-03
 */

#include "../../include/tools/color_terminal.hpp"

namespace multisensor_localization
{
    /**
     * @brief 终端输出提示
     * @note 
     * @todo
     **/
    void ColorTerminal::ColorInfo(std::string str)
    {
        std::cout << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl;
    }



} // namespace multisensor_localization

/*
 * @Description:    Output colorful content to the terminal
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-04
 * @Note:
 */

#include "../../include/tools/ColorTerminal.hpp"
// c++
#include <string>
#include <iostream>

namespace glins
{
    /**
     * @brief  output node layer
     * @note
     * @todo
     **/
    void ColorTerminal::NodeInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorBlueBold << str << fontColorReset << std::endl
                  << std::endl;
    }
    /**
     * @brief  output flow layer
     * @note
     * @todo
     **/
    void ColorTerminal::FlowInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorYellowBold << str << fontColorReset << std::endl
                  << std::endl;
    }

} // namespace glins

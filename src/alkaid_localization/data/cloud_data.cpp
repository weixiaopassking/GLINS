#include "cloud_data.hpp"

namespace data_ns
{
CloudData::CloudData()
{
    _cloud_ptr.reset(new CLOUD);
}
} // namespace data_ns

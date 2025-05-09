#ifndef RMCL_CORRECTION_MODEL_SETTER_HPP
#define RMCL_CORRECTION_MODEL_SETTER_HPP

#include <rmagine/types/sensor_models.h>

namespace rmcl
{

template<typename ModelT>
class ModelSetter
{
public:
  virtual void setModel(const ModelT&) = 0;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_MODEL_SETTER_HPP
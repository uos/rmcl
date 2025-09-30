#include "Input.hpp"

namespace rmcl
{

template<typename DataT>
void Input<DataT>::setInput(const DataT input)
{
  input_ = input;
}

} // namespace rmcl
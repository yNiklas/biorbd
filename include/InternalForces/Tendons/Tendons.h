#ifndef BIORBD_TENDONS_H
#define BIORBD_TENDONS_H

#include "biorbdConfig.h"

#include <memory>
#include <vector>

namespace BIORBD_NAMESPACE {
namespace utils {
class String;
}

namespace internal_forces {
namespace tendons {
class Tendon;
class TendonGeometry;

class BIORBD_API Tendons {
public:
  Tendons();

  Tendons(const Tendons& other);

  Tendons DeepCopy() const;

  void DeepCopy(const Tendons& other);

  void addTendon(
      const utils::String& name,
      const TendonGeometry& geometry);

  std::vector<std::shared_ptr<Tendon>> tendons() const;

  Tendon& tendon(size_t idx) const;

  std::vector<utils::String> tendonNames() const;

  size_t nbTendons() const;

protected:
  std::shared_ptr<std::vector<std::shared_ptr<Tendon>>> m_tendons;
};
}
}
}

#endif  // BIORBD_TENDONS_H

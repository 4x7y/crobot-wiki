#include "crobot/math/random.hpp"

namespace crobot {
namespace math {

//==============================================================================
Random::GeneratorType& Random::getGenerator() {
  static GeneratorType randGenerator(getSeed());
  return randGenerator;
}

//==============================================================================
void Random::setSeed(unsigned int seed) {
  std::seed_seq seq{seed};
  getSeedMutable() = seed;
  getGenerator().seed(seq);
}

//==============================================================================
unsigned int Random::generateSeed(bool applyGeneratedSeed) {
  const unsigned int seed = std::random_device{}();
  if (applyGeneratedSeed)
    setSeed(seed);
  return seed;
}

//==============================================================================
unsigned int Random::getSeed() {
  return getSeedMutable();
}

//==============================================================================
unsigned int& Random::getSeedMutable() {
  static uint32_t seed = generateSeed(false);
  return seed;
}

} // namespace math
} // namespace crobot

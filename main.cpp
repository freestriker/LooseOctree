#include "LooseOctree.hpp"

class TestElement
{

};
struct TestSemantics
{
	static constexpr float LoosenessRatio = 0.125;
	static constexpr uint32_t MaxDepthCount = 3;
};

using TestLooseOctree = LooseOctree<TestElement, TestSemantics>;

int main()
{
	TestLooseOctree octree{glm::vec3(0), 10};
	return  0;
}
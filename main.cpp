#include "LooseOctree.hpp"

struct TestElement
{
	BoxCenterAndExtent bounds;
	ElementId elementId;
};
struct TestSemantics
{
	static constexpr float LoosenessRatio = 0.125;
	static constexpr uint32_t MaxDepthCount = 3;
	static constexpr uint32_t MaxElementsPerLeaf = 1;
	inline static const BoxCenterAndExtent& GetBoundingBox(const TestElement* element)
	{
		return element->bounds;
	}
	inline static void SetElementId(TestElement* element, const ElementId elementId)
	{
		element->elementId = elementId;
	}
};

using TestLooseOctree = LooseOctree<TestElement*, TestSemantics>;

int main()
{
	TestLooseOctree octree{glm::vec3(0), 10};

	TestElement e0{};
	e0.bounds = BoxCenterAndExtent(glm::vec3(0), glm::vec3(10));

	octree.AddElement(&e0);
	
	return  0;
}
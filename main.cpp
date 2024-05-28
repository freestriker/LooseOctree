#include <iostream>

#include "LooseOctree.hpp"

struct TestElement
{
	BoxCenterAndExtent bounds;
	LooseOctreeElementId elementId;
};
struct TestSemantics
{
	static constexpr float LoosenessRatio = 0.5;
	static constexpr uint32_t MaxDepthCount = 3;
	static constexpr uint32_t MaxElementsPerLeaf = 2;
	static constexpr uint32_t MinInclusiveElementsPerNode = 2;
	inline static const BoxCenterAndExtent& GetBoundingBox(const TestElement* element)
	{
		return element->bounds;
	}
	inline static void SetElementId(TestElement* element, const LooseOctreeElementId elementId)
	{
		element->elementId = elementId;
	}
};

using TestLooseOctree = LooseOctree<TestElement*, TestSemantics>;

int main()
{
	TestLooseOctree octree{glm::vec3(0), 4};

	TestElement e0{};
	e0.bounds = BoxCenterAndExtent(glm::vec3(0), glm::vec3(3));
	octree.AddElement(&e0);
	
	TestElement e00{};
	e00.bounds = BoxCenterAndExtent(glm::vec3(0), glm::vec3(3));
	octree.AddElement(&e00);
	
	TestElement e1{};
	e1.bounds = BoxCenterAndExtent(glm::vec3(1), glm::vec3(2));	
	octree.AddElement(&e1);

	TestElement e10{};
	e10.bounds = BoxCenterAndExtent(glm::vec3(1), glm::vec3(2));	
	octree.AddElement(&e10);
	
	TestElement e2{};
	e2.bounds = BoxCenterAndExtent(glm::vec3(1.75), glm::vec3(1));
	octree.AddElement(&e2);

	TestElement e20{};
	e20.bounds = BoxCenterAndExtent(glm::vec3(1.75), glm::vec3(2));
	octree.AddElement(&e20);

	std::cout << "FindAllElements" << std::endl;
	octree.FindAllElements([](const TestElement* element)->void
	{
		std::cout << "nodeIndex: " << element->elementId.nodeIndex << " elementIndex: " << element->elementId.elementIndex <<std::endl;
	});

	std::cout << "FindElementsWithBoundsTest" << std::endl;
	octree.FindElementsWithBoundsTest(BoxCenterAndExtent(glm::vec3(0), glm::vec3(0.5)), [](TestElement* element)->void
	{
		std::cout << "nodeIndex: " << element->elementId.nodeIndex << " elementIndex: " << element->elementId.elementIndex <<std::endl;
	});

	auto&& elements = octree.GetElementsForNode(0);

	octree.RemoveElement(e1.elementId);
	octree.RemoveElement(e20.elementId);
	octree.RemoveElement(e10.elementId);
	octree.RemoveElement(e2.elementId);
	octree.RemoveElement(e0.elementId);
	
	
	
	
	return  0;
}
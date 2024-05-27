﻿#include <iostream>

#include "LooseOctree.hpp"

struct TestElement
{
	BoxCenterAndExtent bounds;
	ElementId elementId;
};
struct TestSemantics
{
	static constexpr float LoosenessRatio = 0.5;
	static constexpr uint32_t MaxDepthCount = 3;
	static constexpr uint32_t MaxElementsPerLeaf = 1;
	static constexpr uint32_t MinInclusiveElementsPerNode = 1;
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
	TestLooseOctree octree{glm::vec3(0), 4};

	TestElement e0{};
	e0.bounds = BoxCenterAndExtent(glm::vec3(0), glm::vec3(3));
	octree.AddElement(&e0);
	
	TestElement e1{};
	e1.bounds = BoxCenterAndExtent(glm::vec3(1), glm::vec3(2));	
	octree.AddElement(&e1);
	
	TestElement e2{};
	e2.bounds = BoxCenterAndExtent(glm::vec3(1.75), glm::vec3(1));
	octree.AddElement(&e2);

	TestElement e20{};
	e20.bounds = BoxCenterAndExtent(glm::vec3(1.75), glm::vec3(2));
	octree.AddElement(&e20);

	octree.FindAllElements([](TestElement* element)->void
	{
		std::cout << "nodeIndex: " << element->elementId.nodeIndex << " elementIndex: " << element->elementId.elementIndex <<std::endl;
	});

	octree.FindElementsWithBoundsTest(BoxCenterAndExtent(glm::vec3(0), glm::vec3(0.5)), [](TestElement* element)->void
	{
		std::cout << "nodeIndex: " << element->elementId.nodeIndex << " elementIndex: " << element->elementId.elementIndex <<std::endl;
	});
	
	octree.RemoveElement(e0.elementId);
	octree.RemoveElement(e1.elementId);
	octree.RemoveElement(e2.elementId);
	octree.RemoveElement(e20.elementId);
	
	return  0;
}
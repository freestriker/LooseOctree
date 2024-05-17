#include "LooseOctree.hpp"

class TestElement
{

};
class TestSemantics
{

};

using TestLooseOctree = LooseOctree<TestElement, TestSemantics>;

int main()
{
	TestLooseOctree octree{};
	return  0;
}
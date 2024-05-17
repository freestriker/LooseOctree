#pragma once

#include <boost/type_traits.hpp>
#include <glm/glm.hpp>
#include <vector>

template<typename TElement, typename TSemantics>
class LooseOctree
{
public:
	using NodeIndex = uint32_t;
	static constexpr uint32_t NONE_NODE_INDEX = uint32_t(-1);
	struct BoxCenterAndExtent
	{
		glm::vec3 center;
		glm::vec3 extent;
	};
private:
	struct NodeContext
	{
		BoxCenterAndExtent bounds;
		float childExtent;
		float childCenterOffset;
		uint32_t level;

	};
	struct Node
	{
		NodeIndex childNodeStartIndex = NONE_NODE_INDEX;
		uint32_t inclusiveElementCount = 0;

		bool IsLeaf() const
		{
			return childNodeStartIndex == NONE_NODE_INDEX;
		}
	};
	struct OffsetAndExtent
	{
		float offset;
		float extent;
	};

	NodeContext rootNodeContext;
	std::vector<Node> treeNodes;
	// Indexed by compacted indexs
	std::vector<NodeIndex> parentNodeIndexs;
	std::vector<std::vector<TElement>> elementVectors;
	// Compacted indexs
	std::vector<NodeIndex> freeNodeStartIndexs;
	std::vector<OffsetAndExtent> levelOffsetAndExtents;

	static inline NodeIndex ToCompactNodeIndex(const NodeIndex nodeIndex)
	{
		// 1 for the existed root
		return (nodeIndex - 1) / 8;
	}

	static inline NodeIndex FromCompactNodeIndex(const NodeIndex compactNodeIndex)
	{
		// 1 for the existed root
		return compactNodeIndex * 8 + 1;
	}

	NodeIndex AllocateEightNodes()
	{
		NodeIndex nodeStartIndex = NONE_NODE_INDEX;
		if (freeNodeStartIndexs.empty())
		{
			nodeStartIndex = static_cast<uint32_t>(treeNodes.size());

			treeNodes.insert(treeNodes.end(), 8, Node(NONE_NODE_INDEX, 0));
			parentNodeIndexs.emplace_back(NONE_NODE_INDEX);
			elementVectors.insert(elementVectors.end(), 8, {});
		}
		else
		{
			nodeStartIndex = FromCompactNodeIndex(freeNodeStartIndexs.back());

			freeNodeStartIndexs.pop_back();
		}
		return nodeStartIndex;
	}

	void FreeEightNodes(const NodeIndex nodeStartIndex)
	{
		std::fill_n(treeNodes.begin() + nodeStartIndex, 8, Node(NONE_NODE_INDEX, 0));
		parentNodeIndexs[ToCompactNodeIndex(nodeStartIndex)] = NONE_NODE_INDEX;
		freeNodeStartIndexs.emplace_back(ToCompactNodeIndex(nodeStartIndex));
	}

};
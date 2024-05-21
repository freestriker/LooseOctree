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

		BoxCenterAndExtent(const glm::vec3& center, const glm::vec3& extent)
			: center(center)
			, extent(extent)
		{
			
		}
	};
private:
	struct NodeContext
	{
		BoxCenterAndExtent bounds;
		uint32_t level;

		NodeContext(const BoxCenterAndExtent& box, const uint32_t level = 0)
			: bounds(box)
			, level(level)
		{
			
		}
	};
	struct Node
	{
		NodeIndex childNodeStartIndex;
		uint32_t inclusiveElementCount;

		bool IsLeaf() const
		{
			return childNodeStartIndex == NONE_NODE_INDEX;
		}

		Node()
			: childNodeStartIndex(NONE_NODE_INDEX)
			, inclusiveElementCount(0)
		{
			
		}
	};
	struct OffsetAndExtent
	{
		float offset;
		float extent;
		
		OffsetAndExtent(const float offset, const float extent)
			: offset(offset)
			, extent(extent)
		{
			
		}
		OffsetAndExtent()
			: offset(0)
			, extent(0)
		{
			
		}
	};

	NodeContext rootNodeContext;
	std::vector<Node> treeNodes;
	// Indexed by compacted indexs
	std::vector<NodeIndex> parentNodeIndexs;
	std::vector<std::vector<TElement>> elementVectors;
	// Compacted indexs
	std::vector<NodeIndex> freeNodeStartIndexs;
	const std::vector<OffsetAndExtent> levelOffsetAndExtents;

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

	static inline std::vector<OffsetAndExtent> BuildOffsetAndExtents(const float extent)
	{
		float parentExtent = extent;
		
		std::vector<OffsetAndExtent> offsetAndExtents{ TSemantics::MaxDepthCount };
		offsetAndExtents[0] = OffsetAndExtent(0, extent);
		for(uint32_t depthIndex = 1; depthIndex < TSemantics::MaxDepthCount; ++depthIndex)
		{
			const float tightChildExtent = parentExtent * 0.5f;
			const float looseChildExtent = tightChildExtent * (1.0f + TSemantics::LoosenessRatio);

			offsetAndExtents[depthIndex] = OffsetAndExtent(parentExtent - looseChildExtent, looseChildExtent);

			parentExtent = looseChildExtent;
		}

		return offsetAndExtents;
	}
public:
	LooseOctree(const glm::vec3& center, const float extent)
		: rootNodeContext(BoxCenterAndExtent(center, glm::vec3(extent)))
		, treeNodes({ Node() })
		, parentNodeIndexs()
		, elementVectors({ {} })
		, freeNodeStartIndexs()
		, levelOffsetAndExtents(BuildOffsetAndExtents(extent))
	{
		
	}

};
--!strict
-- Services
-- Packages
local Package = script.Parent
local Packages = Package.Parent
local TableUtil = require(Packages:WaitForChild("TableUtil"))
-- Modules
local Pathfinder = require(script.Parent:WaitForChild("Pathfinder"))
-- Types
type List<V> = TableUtil.List<V>
type Dict<K, V> = TableUtil.Dict<K,V>
type PathfinderProperties<T, Cache> = Pathfinder.PathfinderProperties<T, Cache>
type PathfinderFunctions<Self, Cache, G> = Pathfinder.PathfinderFunctions<Self, Cache, G>
export type ScoreFunction<T> = Pathfinder.ScoreFunction<T>
export type PathNetwork<T> = Pathfinder.PathNetwork<T>
export type PathList<T> = Pathfinder.PathList<T>
export type WeightedNetwork<T> = Dict<T, Dict<T, number>>
export type DistanceCache<T> = Dict<T, Dict<T, number>>
export type PathCache<T> = Dict<T, Dict<T, PathList<T>>>

export type DijkstraCache<T> = {
	Path: PathCache<T>,
	Distance: DijkstraCache<T>,
}
export type DijkstraProperties<T> = {
	_WeightedNetwork: WeightedNetwork<T>,
} & PathfinderProperties<T, DijkstraCache<T>>

export type DijkstraFunctions<Self, G> = {
} & PathfinderFunctions<Self, DijkstraCache<G>, G>

type BaseDijkstra<Self, T> = DijkstraProperties<T> & DijkstraFunctions<Self, T>
export type Dijkstra<T> = BaseDijkstra<BaseDijkstra<any, T>, T>
-- Constants
-- Variables
-- References
-- Private functions
function getWeightedNetwork<T>(network: PathNetwork<T>, scoreFunction: ScoreFunction<T>): WeightedNetwork<T>
	local weightedNetwork = {}
	for s, connections in pairs(network) do

		for i, d in ipairs(connections) do

			weightedNetwork[d] = weightedNetwork[d] or {}
			weightedNetwork[d][s] = scoreFunction(s, d)
		end
	end
	return weightedNetwork
end

local function getScoredDistances<T>(goal: T, weightedNetwork: WeightedNetwork<T>): {[T]: number}
	
	local distances: {[T]: number} = {}
	distances[goal] = 0
	local queue = {}
	for n, _ in pairs(weightedNetwork) do
		if n ~= goal then
			distances[n] = math.huge
			table.insert(queue, n)
		end
	end

	local function scoreStep(node: T, prevDist: number)
		if node == goal then return end
		if distances[node] and distances[node] > prevDist then
			distances[node] = prevDist
			for n, dist in pairs(weightedNetwork[node]) do
				scoreStep(n, prevDist+dist)
			end
		end
	end

	for n: T, dist: number in pairs(weightedNetwork[goal]) do
		scoreStep(n, dist)
	end

	return distances
end


function solve<T>(
	start: T, 
	finish: T, 
	network: PathNetwork<T>, 
	cache: DijkstraCache<T>, 
	weightedNetwork: WeightedNetwork<T>
): (PathList<T>?, DijkstraCache<T>?)
	local path: {[number]: T} = {}

	local pathCache, distanceCache = cache.Path, cache.Distance

	--checks long term cache for existing solutions, otherwise solves from scratch and stores for later.
	pathCache[start] = pathCache[start] or {}
	if pathCache[start][finish] ~= nil then
		for i, node in ipairs(pathCache[start][finish]) do
			table.insert(path, node)
		end
	else
		--checks the distance cache to see if it needs to rescore the distances
		if distanceCache[finish] == nil then
			distanceCache[finish] = getScoredDistances(finish, weightedNetwork)
		end
		local distances: {[T]: number} = distanceCache[finish]

		--assembles the path by following the lowest distance scoring connections to the goal.
		local function assemblePath(node: T)
			table.insert(path, node)
			table.insert(pathCache[start][finish], node)
			if node == finish then return end
			local closestDist = math.huge
			local closestNode = nil
			for i, otherNode: T in ipairs(network[node] or {}) do
				if table.find(path, otherNode) == nil then
					local otherDist = distances[otherNode]
					if otherDist < closestDist then
						closestNode = otherNode
						closestDist = otherDist
					end
				end	
			end
			if closestNode then
				assemblePath(closestNode)
			end
		end

		pathCache[start][finish] = {}
		assemblePath(start)

		--takes solved path, reverses it for caching
		pathCache[finish] = pathCache[finish] or {}
		pathCache[finish][start] = TableUtil.reverse(path)
	end
	if path[1] == start and path[#path] == finish then
		return path, cache
	end
	return nil, nil
end

-- Class
local Dijkstra: Dijkstra<any> = {} :: any
Dijkstra.__index = Dijkstra
setmetatable(Dijkstra, Pathfinder)

function Dijkstra:Solve(start: any, finish: any): PathList<any>?
	local pathList, cache = solve(
		start, 
		finish, 
		self._Network, 
		self._Cache, 
		self._WeightedNetwork
	)
	if pathList and cache then
		self:SetCache(cache)
		return pathList
	end
	return nil
end

function Dijkstra:SetNetwork(network: PathNetwork<any>): nil
	Pathfinder.SetNetwork(self, network)
	self._WeightedNetwork = getWeightedNetwork(network, self._ScoreFunction)
	return nil
end

function Dijkstra:ClearCache()
	self._Cache = {
		Distance = {},
		Path = {},
	} :: DijkstraCache<any>
	return nil
end

function Dijkstra.new(initialNetwork: PathNetwork<any>, scoreFunction: ScoreFunction<any>): Dijkstra<any>

	local self: Dijkstra<any> = setmetatable(Pathfinder.new(initialNetwork, scoreFunction), Dijkstra) :: any
	self._Cache = {
		Distance = {},
		Path = {},
	} :: DijkstraCache<any>
	self:SetNetwork(initialNetwork)

	return self
end

return Dijkstra
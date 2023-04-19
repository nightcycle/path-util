--!strict
-- Services
-- Packages
local Package = script.Parent
local Packages = Package.Parent
local TableUtil = require(Packages:WaitForChild("TableUtil"))
-- Modules
-- Types
type List<V> = TableUtil.List<V>
type Dict<K, V> = TableUtil.Dict<K,V>

export type ScoreFunction<T> = (a: T, b: T) -> number
export type PathNetwork<T> = Dict<T, List<T>>
export type PathList<T> = List<T>

export type PathfinderProperties<T, Cache> = {
	_Cache: Cache,
	_Network: PathNetwork<T>,
}

export type PathfinderFunctions<Self, Cache, G> = {
	__index: Self,
	_Network: PathNetwork<G>,
	_ScoreFunction: ScoreFunction<G>,
	ClearCache: (self: Self) -> nil,
	SetCache: (self: Self, cache: Cache) -> nil,
	GetCache: (self: Self) -> Cache,
	SetScoreFunction: (self: Self, ScoreFunction<G>) -> nil,
	SetNetwork: (self: Self, network: PathNetwork<G>) -> nil, 
	Score: (self: Self, a: G, b: G) -> number,
	Solve: (self: Self, start: G, finish: G) -> PathList<G>?,
	new: (initialNetwork: PathNetwork<G>, scoreFunction: ScoreFunction<G>) -> Self,
}

type BasePathfinder<Self, C, T> = PathfinderProperties<T, C> & PathfinderFunctions<Self, C, T>
export type Pathfinder<T, Cache> = BasePathfinder<BasePathfinder<any, Cache, T>, Cache, T>

-- Constants
-- Variables
-- References
-- Class
local Pathfinder: Pathfinder<any, any> = {} :: any
Pathfinder.__index = Pathfinder

function Pathfinder:Solve(start: any, finish: any): PathList<any>?
	return nil
end

function Pathfinder:ClearCache(): nil
	self._Cache = {}
	return nil
end

function Pathfinder:SetCache(cache: any): nil
	self._Cache = cache
	return nil
end

function Pathfinder:GetCache(cache: any): nil
	return self._Cache
end

function Pathfinder:SetNetwork(network: PathNetwork<any>): nil
	self:ClearCache()
	self._Network = network
	return nil
end


function Pathfinder:Score(a: any, b: any): number
	return self._ScoreFunction(a,b)
end

function Pathfinder.new(initialNetwork: PathNetwork<any>, scoreFunction: ScoreFunction<any>)

	local self: Pathfinder<any, any> = setmetatable({}, Pathfinder) :: any
	self._ScoreFunction = scoreFunction
	self._Cache = {}
	self:SetNetwork(initialNetwork)

	return self
end

return Pathfinder
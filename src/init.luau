--!strict
-- Services
-- Packages
local Package = script
local Packages = Package.Parent
local TableUtil = require(Packages:WaitForChild("TableUtil"))
-- Gamework
-- Modules
local Dijkstra = require(script:WaitForChild("Dijkstra"))
-- Types
type List<V> = TableUtil.List<V>
type Dict<K, V> = TableUtil.Dict<K,V>
export type Dijkstra<T> = Dijkstra.Dijkstra<T>
export type ScoreFunction<T> = Dijkstra.ScoreFunction<T>
export type PathNetwork<T> = Dict<T, List<T>>
export type PathList<T> = List<T>
export type DijkstraCache<T> = Dijkstra.DijkstraCache<T>
-- Constants
-- Variables
-- References
-- Class
local Pathfind = {}
Pathfind.__index = Pathfind

function Pathfind.dijkstra<T>(initialNetwork: PathNetwork<T>, scoreFunction: ScoreFunction<T>): Dijkstra<T>
	return Dijkstra.new(initialNetwork, scoreFunction)
end

return Pathfind
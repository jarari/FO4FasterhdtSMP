#pragma once

namespace Smp::Game
{
	using FixedString = RE::BSFixedString;
	using Node = RE::NiNode;
	using Object = RE::NiAVObject;
	using Geometry = RE::BSGeometry;
	using TriShape = RE::BSTriShape;

	template <class T>
	using NiPtr = RE::NiPointer<T>;

	template <class T>
	using SmartPtr = RE::BSTSmartPointer<T>;

	template <class T>
	[[nodiscard]] NiPtr<T> MakeNiPtr(T* a_ptr)
	{
		NiPtr<T> result;
		result.reset(a_ptr);
		return result;
	}

	template <class T>
	[[nodiscard]] SmartPtr<T> MakeSmartPtr(T* a_ptr)
	{
		SmartPtr<T> result;
		result.reset(a_ptr);
		return result;
	}
}

#pragma once

namespace Smp
{
	namespace ImguiLayer
	{
		using DrawCallback = void (*)();

		bool Initialize();
		void RenderFrame();
		bool IsInitialized();
		bool IsOpen();
		void SetEnabled(bool a_enabled);
		void SetDrawCallback(DrawCallback a_callback);
	}
}

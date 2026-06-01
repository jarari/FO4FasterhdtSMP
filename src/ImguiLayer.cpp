#include "ImguiLayer.h"

#include "RE/B/BSGraphics.h"

#if defined(_M_X64) && !defined(_AMD64_)
#	define _AMD64_ 1
#endif
#include <Windows.h>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND a_hwnd, UINT a_msg, WPARAM a_wparam, LPARAM a_lparam);

namespace
{
	constexpr WPARAM kToggleKey = VK_INSERT;

	bool                            Initialized{ false };
	bool                            Enabled{ false };
	bool                            MenuOpen{ false };
	bool                            WindowActive{ true };
	bool                            PresentHookInstalled{ false };
	bool                            RenderingFromPresent{ false };
	WNDPROC                         OriginalWndProc{ nullptr };
	HWND                            WindowHandle{ nullptr };
	Smp::ImguiLayer::DrawCallback   DebugDrawCallback{ nullptr };
	RE::BSGraphics::RendererData*   RendererData{ nullptr };
	using Present_t = HRESULT(STDMETHODCALLTYPE*)(REX::W32::IDXGISwapChain*, std::uint32_t, std::uint32_t);
	Present_t OriginalPresent{ nullptr };

	bool IsAltTabSystemKey(const UINT a_msg, const WPARAM a_wparam)
	{
		return (a_msg == WM_SYSKEYDOWN || a_msg == WM_SYSKEYUP) &&
			(a_wparam == VK_TAB || a_wparam == VK_MENU || a_wparam == VK_LMENU || a_wparam == VK_RMENU);
	}

	LRESULT CallOriginalWndProc(HWND a_hwnd, UINT a_msg, WPARAM a_wparam, LPARAM a_lparam)
	{
		return OriginalWndProc ? CallWindowProc(OriginalWndProc, a_hwnd, a_msg, a_wparam, a_lparam) : DefWindowProc(a_hwnd, a_msg, a_wparam, a_lparam);
	}

	LRESULT CALLBACK WndProc(HWND a_hwnd, UINT a_msg, WPARAM a_wparam, LPARAM a_lparam)
	{
		switch (a_msg) {
		case WM_KEYDOWN:
			if ((a_lparam & 0x40000000) == 0 && a_wparam == kToggleKey) {
				MenuOpen = !MenuOpen;
				::ShowCursor(MenuOpen ? TRUE : FALSE);
			}
			break;
		case WM_ACTIVATEAPP:
			WindowActive = a_wparam != FALSE;
			if (!WindowActive) {
				::ClipCursor(nullptr);
			}
			break;
		case WM_ACTIVATE:
			WindowActive = LOWORD(a_wparam) != WA_INACTIVE;
			if (!WindowActive) {
				::ClipCursor(nullptr);
			}
			break;
		case WM_KILLFOCUS:
			WindowActive = false;
			::ClipCursor(nullptr);
			break;
		case WM_SETFOCUS:
			WindowActive = true;
			break;
		default:
			break;
		}

		if (Initialized && MenuOpen) {
			ImGui_ImplWin32_WndProcHandler(a_hwnd, a_msg, a_wparam, a_lparam);

			switch (a_msg) {
			case WM_ACTIVATE:
			case WM_ACTIVATEAPP:
			case WM_KILLFOCUS:
			case WM_SETFOCUS:
			case WM_SIZE:
			case WM_SYSCOMMAND:
				return CallOriginalWndProc(a_hwnd, a_msg, a_wparam, a_lparam);
			case WM_SYSKEYDOWN:
			case WM_SYSKEYUP:
				return IsAltTabSystemKey(a_msg, a_wparam) ? CallOriginalWndProc(a_hwnd, a_msg, a_wparam, a_lparam) : 0;
			case WM_KEYDOWN:
			case WM_KEYUP:
			case WM_CHAR:
			case WM_DEADCHAR:
			case WM_SYSCHAR:
			case WM_SYSDEADCHAR:
			case WM_INPUT:
			case WM_MOUSEMOVE:
			case WM_LBUTTONDOWN:
			case WM_LBUTTONUP:
			case WM_LBUTTONDBLCLK:
			case WM_RBUTTONDOWN:
			case WM_RBUTTONUP:
			case WM_RBUTTONDBLCLK:
			case WM_MBUTTONDOWN:
			case WM_MBUTTONUP:
			case WM_MBUTTONDBLCLK:
			case WM_XBUTTONDOWN:
			case WM_XBUTTONUP:
			case WM_XBUTTONDBLCLK:
			case WM_MOUSEWHEEL:
			case WM_MOUSEHWHEEL:
				return 0;
			default:
				break;
			}
		}

		return CallOriginalWndProc(a_hwnd, a_msg, a_wparam, a_lparam);
	}

	void DrawControlWindow()
	{
		ImGui::SetNextWindowPos(ImVec2(24.0F, 24.0F), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(360.0F, 180.0F), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowBgAlpha(0.88F);

		if (ImGui::Begin("FO4 Faster HDT-SMP Debug")) {
			ImGui::TextUnformatted("ImGui layer active");
			ImGui::Separator();
			ImGui::Text("RendererData: %p", static_cast<void*>(RendererData));
			ImGui::Text("Device:       %p", RendererData ? static_cast<void*>(RendererData->device) : nullptr);
			ImGui::Text("Context:      %p", RendererData ? static_cast<void*>(RendererData->context) : nullptr);
			ImGui::Text("Window active: %s", WindowActive ? "yes" : "no");
			ImGui::TextUnformatted("Bullet debug drawing can use ImGui foreground/background draw lists from the registered callback.");
		}
		ImGui::End();
	}

	bool InstallVTableSlot(void** a_vtable, const std::size_t a_index, void* a_hook, void*& a_original)
	{
		if (!a_vtable || !a_hook || a_original) {
			return false;
		}

		DWORD oldProtect = 0;
		auto* slot = std::addressof(a_vtable[a_index]);
		if (!::VirtualProtect(slot, sizeof(void*), PAGE_EXECUTE_READWRITE, std::addressof(oldProtect))) {
			return false;
		}

		a_original = *slot;
		*slot = a_hook;

		DWORD restoredProtect = 0;
		::VirtualProtect(slot, sizeof(void*), oldProtect, std::addressof(restoredProtect));
		return true;
	}

	HRESULT STDMETHODCALLTYPE HookedPresent(REX::W32::IDXGISwapChain* a_swapChain, const std::uint32_t a_syncInterval, const std::uint32_t a_flags)
	{
		RenderingFromPresent = true;
		Smp::ImguiLayer::RenderFrame();
		RenderingFromPresent = false;

		return OriginalPresent ? OriginalPresent(a_swapChain, a_syncInterval, a_flags) : E_FAIL;
	}

	void EnsurePresentHook()
	{
		if (PresentHookInstalled || !RendererData) {
			return;
		}

		auto* swapChain = RendererData->renderWindow[0].swapChain;
		if (!swapChain) {
			return;
		}

		auto* vtable = *reinterpret_cast<void***>(swapChain);
		void* original = nullptr;
		if (!InstallVTableSlot(vtable, 8, reinterpret_cast<void*>(HookedPresent), original)) {
			spdlog::warn("failed to install ImGui Present hook");
			return;
		}

		OriginalPresent = reinterpret_cast<Present_t>(original);
		PresentHookInstalled = true;
		spdlog::info("FO4 Faster HDT-SMP ImGui Present hook installed");
	}
}

namespace Smp::ImguiLayer
{
	bool Initialize()
	{
		if (Initialized) {
			return true;
		}

		RendererData = RE::BSGraphics::GetRendererData();
		if (!RendererData || !RendererData->device || !RendererData->context) {
			return false;
		}

		WindowHandle = reinterpret_cast<HWND>(RendererData->renderWindow[0].hwnd);
		if (!WindowHandle) {
			return false;
		}

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		auto& io = ImGui::GetIO();
		io.IniFilename = nullptr;
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

		OriginalWndProc = reinterpret_cast<WNDPROC>(SetWindowLongPtr(WindowHandle, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(WndProc)));
		if (!OriginalWndProc) {
			spdlog::warn("failed to install ImGui WndProc hook");
			return false;
		}

		ImGui_ImplWin32_Init(WindowHandle);
		ImGui_ImplDX11_Init(
			reinterpret_cast<ID3D11Device*>(RendererData->device),
			reinterpret_cast<ID3D11DeviceContext*>(RendererData->context));

		Initialized = true;
		EnsurePresentHook();
		spdlog::info("FO4 Faster HDT-SMP ImGui layer initialized");
		return true;
	}

	void RenderFrame()
	{
		if (!Enabled) {
			return;
		}

		if (!Initialized && !Initialize()) {
			return;
		}
		EnsurePresentHook();
		if (PresentHookInstalled && !RenderingFromPresent) {
			return;
		}

		ImGui_ImplDX11_NewFrame();
		ImGui_ImplWin32_NewFrame();
		ImGui::NewFrame();

		if (MenuOpen) {
			DrawControlWindow();
		}
		if (DebugDrawCallback) {
			DebugDrawCallback();
		}

		ImGui::Render();

		auto* context = RendererData ? RendererData->context : nullptr;
		if (!context) {
			return;
		}

		REX::W32::ID3D11RenderTargetView* oldRtv{ nullptr };
		REX::W32::ID3D11DepthStencilView* oldDsv{ nullptr };
		context->OMGetRenderTargets(1, std::addressof(oldRtv), std::addressof(oldDsv));

		auto* backBufferRtv = RendererData->renderWindow[0].swapChainRenderTarget.rtView;
		if (backBufferRtv) {
			context->OMSetRenderTargets(1, std::addressof(backBufferRtv), nullptr);
		} else {
			static bool loggedMissingBackBuffer = false;
			if (!loggedMissingBackBuffer) {
				spdlog::warn("ImGui render skipped explicit backbuffer bind because swapChainRenderTarget.rtView is null");
				loggedMissingBackBuffer = true;
			}
		}

		ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

		context->OMSetRenderTargets(1, std::addressof(oldRtv), oldDsv);
		if (oldRtv) {
			oldRtv->Release();
		}
		if (oldDsv) {
			oldDsv->Release();
		}
	}

	bool IsInitialized()
	{
		return Initialized;
	}

	bool IsOpen()
	{
		return MenuOpen;
	}

	void SetEnabled(const bool a_enabled)
	{
		Enabled = a_enabled;
		if (!Enabled) {
			MenuOpen = false;
		}
	}

	void SetDrawCallback(const DrawCallback a_callback)
	{
		DebugDrawCallback = a_callback;
	}
}

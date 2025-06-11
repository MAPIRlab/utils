#pragma once
#include "PID.h"
#include <imgui_gl/imgui_gl.h>

inline void RenderPIDGUI(PID& pid, const std::string& name)
{
    ImGui::Begin(name.c_str());
    ImGui::InputFloat("P", &pid.kP);
    ImGui::InputFloat("I", &pid.kI);
    ImGui::InputFloat("D", &pid.kD);
    ImGui::InputFloat("Max Output", &pid.maximumOutput);
    ImGui::End();
}
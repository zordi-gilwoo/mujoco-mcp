@echo off
:: MuJoCo-MCP 安装脚本 (Windows版)
:: 此脚本帮助用户安装MuJoCo-MCP及其依赖项

echo === MuJoCo-MCP 安装脚本 (Windows版) ===
echo.
echo 此脚本将：
echo 1. 检查Python环境
echo 2. 安装MuJoCo依赖
echo 3. 安装MCP (Model Context Protocol)
echo 4. 以开发模式安装MuJoCo-MCP
echo.

:: 确认是否继续
set /p response=是否继续安装? [Y/n] 
if /i "%response%"=="n" goto :cancel
if /i "%response%"=="no" goto :cancel

:: 获取脚本所在目录的绝对路径
set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%.."

echo === 检查Python环境 ===
:: 检查Python是否已安装
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo 错误: 未找到Python
    echo 请安装Python 3.8或更高版本: https://www.python.org/downloads/
    goto :end
)

:: 检查Python版本
for /f "tokens=2" %%a in ('python --version 2^>^&1') do set "python_version=%%a"
echo 检测到Python版本: %python_version%

:: 检查pip是否已安装
python -m pip --version >nul 2>&1
if %errorlevel% neq 0 (
    echo 错误: 未找到pip
    echo 请安装pip: https://pip.pypa.io/en/stable/installation/
    goto :end
)

echo === 创建虚拟环境 ===
echo 推荐在虚拟环境中安装

:: 询问是否创建虚拟环境
set /p create_venv=创建虚拟环境? [Y/n] 
if /i "%create_venv%"=="n" goto :skip_venv
if /i "%create_venv%"=="no" goto :skip_venv

:: 检查venv模块
python -c "import venv" >nul 2>&1
if %errorlevel% neq 0 (
    echo 错误: Python venv模块未安装
    echo 请先安装venv模块
    goto :end
)

:: 创建虚拟环境
echo 在 %REPO_ROOT%\venv 创建虚拟环境...
python -m venv "%REPO_ROOT%\venv"

:: 激活虚拟环境
echo 激活虚拟环境...
call "%REPO_ROOT%\venv\Scripts\activate.bat"

echo 虚拟环境已创建并激活
goto :venv_done

:skip_venv
echo 跳过创建虚拟环境

:venv_done

echo === 升级pip和安装wheel ===
python -m pip install --upgrade pip wheel

echo === 安装MuJoCo依赖 ===
echo 安装MuJoCo...
python -m pip install mujoco>=2.3.0

:: 检查MuJoCo是否安装成功
python -c "import mujoco; print(f'MuJoCo {mujoco.__version__} 已安装')" >nul 2>&1
if %errorlevel% equ 0 (
    echo MuJoCo安装成功
) else (
    echo 警告: MuJoCo安装可能有问题
    echo 请参考MuJoCo文档: https://github.com/deepmind/mujoco
)

echo === 安装Model Context Protocol (MCP) ===
python -m pip install model-context-protocol>=0.1.0

:: 检查MCP是否安装成功
python -c "import mcp; print('MCP已安装')" >nul 2>&1
if %errorlevel% equ 0 (
    echo MCP安装成功
) else (
    echo 警告: MCP安装可能有问题
)

echo === 安装MuJoCo-MCP ===
cd "%REPO_ROOT%"
python -m pip install -e .

:: 检查MuJoCo-MCP是否安装成功
python -c "import mujoco_mcp; print('MuJoCo-MCP已安装')" >nul 2>&1
if %errorlevel% equ 0 (
    echo MuJoCo-MCP安装成功
) else (
    echo 警告: MuJoCo-MCP安装可能有问题
)

echo === 安装可选依赖 ===
:: 询问是否安装Anthropic API
set /p install_anthropic=安装Anthropic API用于LLM示例? [Y/n] 
if /i "%install_anthropic%"=="n" goto :skip_anthropic
if /i "%install_anthropic%"=="no" goto :skip_anthropic

python -m pip install anthropic
echo Anthropic API已安装
goto :anthropic_done

:skip_anthropic
echo 跳过安装Anthropic API

:anthropic_done

echo === 运行验证 ===
:: 询问是否运行验证脚本
set /p run_verify=运行项目验证脚本? [Y/n] 
if /i "%run_verify%"=="n" goto :skip_verify
if /i "%run_verify%"=="no" goto :skip_verify

python "%REPO_ROOT%\tools\project_verify.py"
goto :verify_done

:skip_verify
echo 跳过项目验证

:verify_done

echo.
echo === 安装完成 ===
echo.
echo 要开始使用MuJoCo-MCP，请尝试运行演示：
echo python %REPO_ROOT%\examples\demo.py
echo.
echo 或者运行LLM集成示例：
echo python %REPO_ROOT%\examples\comprehensive_llm_example.py
echo.

if /i not "%create_venv%"=="n" if /i not "%create_venv%"=="no" (
    echo 注意: 虚拟环境已激活。要在新的命令提示符中使用，请运行:
    echo call "%REPO_ROOT%\venv\Scripts\activate.bat"
)

goto :end

:cancel
echo 安装已取消

:end
pause 
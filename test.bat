          
          :: Create isolated build workspace
          mkdir "C:\build\src"

          :: Copy files except .pixi, pixi.toml, .git
          robocopy . "C:\build\src" /E /XD .pixi .git /XF pixi.toml

          :: Copy conda recipe
          robocopy conda "C:\build\conda" /E
          
          call %CONDA%\condabin\conda.bat activate base
          :: Get release version from GitHub, strip leading 'v'
          FOR /F "delims=" %%A IN ('gh release view --json tagName --jq ".tagName"') DO (
            SET "RELEASE_VERSION=%%A"
          )
          SET "RELEASE_VERSION=%RELEASE_VERSION:v=%"
          :: Set SRC_PATH
          SET "SRC_PATH=C:\build\src"
          :: Run the build
          conda build C:\build\conda --croot "C:\build" -c https://prefix.dev/robostack-jazzy -c conda-forge
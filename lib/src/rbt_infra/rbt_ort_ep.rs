use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use ort::{ep, ep::ExecutionProviderDispatch, session::builder::SessionBuilder};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum OrtExecutionProvider {
    CoreML,
    OpenVINO,
    TensorRT,
    CUDA,
    CPU,
}

impl OrtExecutionProvider {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::CoreML => "CoreML",
            Self::OpenVINO => "OpenVINO",
            Self::TensorRT => "TensorRT",
            Self::CUDA => "CUDA",
            Self::CPU => "CPU",
        }
    }
}

pub fn default_execution_provider() -> OrtExecutionProvider {
    default_execution_provider_for_target()
}

#[cfg(target_os = "macos")]
fn default_execution_provider_for_target() -> OrtExecutionProvider {
    OrtExecutionProvider::CoreML
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn default_execution_provider_for_target() -> OrtExecutionProvider {
    if linux_aarch64_tensorrt_stack_available() {
        OrtExecutionProvider::TensorRT
    } else if linux_aarch64_cuda_stack_available() {
        OrtExecutionProvider::CUDA
    } else {
        OrtExecutionProvider::CPU
    }
}

#[cfg(all(
    not(target_os = "macos"),
    target_arch = "x86_64",
    any(target_os = "linux", target_os = "windows")
))]
fn default_execution_provider_for_target() -> OrtExecutionProvider {
    OrtExecutionProvider::OpenVINO
}

#[cfg(not(any(
    target_os = "macos",
    all(target_os = "linux", target_arch = "aarch64"),
    all(
        target_arch = "x86_64",
        any(target_os = "linux", target_os = "windows")
    )
)))]
fn default_execution_provider_for_target() -> OrtExecutionProvider {
    OrtExecutionProvider::CPU
}

pub fn resolve_execution_provider(configured: &str) -> Option<OrtExecutionProvider> {
    match configured.trim().to_ascii_lowercase().as_str() {
        "" | "auto" => Some(default_execution_provider()),
        "coreml" => Some(OrtExecutionProvider::CoreML),
        "openvino" => Some(OrtExecutionProvider::OpenVINO),
        "tensorrt" => Some(OrtExecutionProvider::TensorRT),
        "cuda" => Some(OrtExecutionProvider::CUDA),
        "cpu" => Some(OrtExecutionProvider::CPU),
        _ => None,
    }
}

pub fn configure_session_builder(
    session_builder: SessionBuilder,
    configured_ep: &str,
    engine_cache_path: &str,
) -> RbtResult<(SessionBuilder, OrtExecutionProvider)> {
    let configured_ep = configured_ep.trim();
    if configured_ep.is_empty() || configured_ep.eq_ignore_ascii_case("auto") {
        return configure_auto_session_builder(session_builder, engine_cache_path);
    }

    let provider = resolve_execution_provider(configured_ep)
        .ok_or_else(|| RbtError::UnsupportedExecutionProvider(configured_ep.to_string()))?;

    let session_builder = match provider {
        OrtExecutionProvider::CoreML => configure_coreml(session_builder, engine_cache_path)?,
        OrtExecutionProvider::OpenVINO => configure_openvino(session_builder)?,
        OrtExecutionProvider::TensorRT => {
            configure_tensorrt_strict(session_builder, engine_cache_path)?
        }
        OrtExecutionProvider::CUDA => configure_cuda_strict(session_builder)?,
        OrtExecutionProvider::CPU => configure_cpu(session_builder)?,
    };

    Ok((session_builder, provider))
}

#[cfg(target_os = "macos")]
fn configure_coreml(
    session_builder: SessionBuilder,
    model_cache_dir: &str,
) -> RbtResult<SessionBuilder> {
    Ok(
        session_builder.with_execution_providers([ep::CoreMLExecutionProvider::default()
            .with_compute_units(ep::coreml::ComputeUnits::All)
            .with_model_format(ep::coreml::ModelFormat::MLProgram)
            .with_specialization_strategy(ep::coreml::SpecializationStrategy::FastPrediction)
            .with_static_input_shapes(true)
            .with_model_cache_dir(model_cache_dir)
            .build()
            .error_on_failure()])?,
    )
}

#[cfg(not(target_os = "macos"))]
fn configure_coreml(
    _session_builder: SessionBuilder,
    _model_cache_dir: &str,
) -> RbtResult<SessionBuilder> {
    Err(RbtError::UnsupportedExecutionProvider(
        "CoreML is only supported on macOS targets".to_string(),
    ))
}

#[cfg(all(
    target_arch = "x86_64",
    any(target_os = "linux", target_os = "windows")
))]
fn configure_openvino(session_builder: SessionBuilder) -> RbtResult<SessionBuilder> {
    Ok(
        session_builder.with_execution_providers([ep::OpenVINOExecutionProvider::default()
            .with_device_type("GPU")
            .build()
            .error_on_failure()])?,
    )
}

#[cfg(not(all(
    target_arch = "x86_64",
    any(target_os = "linux", target_os = "windows")
)))]
fn configure_openvino(_session_builder: SessionBuilder) -> RbtResult<SessionBuilder> {
    Err(RbtError::UnsupportedExecutionProvider(
        "OpenVINO is only enabled for x86_64 Linux/Windows targets".to_string(),
    ))
}

fn configure_cpu(session_builder: SessionBuilder) -> RbtResult<SessionBuilder> {
    Ok(
        session_builder.with_execution_providers([ep::CPUExecutionProvider::default()
            .with_arena_allocator(true)
            .build()])?,
    )
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn try_configure_execution_provider(
    session_builder: SessionBuilder,
    execution_provider: ExecutionProviderDispatch,
) -> Result<SessionBuilder, SessionBuilder> {
    match session_builder.with_execution_providers([execution_provider]) {
        Ok(session_builder) => Ok(session_builder),
        Err(err) => Err(err.recover()),
    }
}

fn tensorrt_execution_provider(engine_cache_path: &str) -> ExecutionProviderDispatch {
    ep::TensorRTExecutionProvider::default()
        .with_engine_cache(true)
        .with_engine_cache_path(engine_cache_path)
        .with_fp16(true)
        .build()
        .error_on_failure()
}

#[cfg(any(
    all(
        target_os = "linux",
        any(target_arch = "aarch64", target_arch = "x86_64")
    ),
    all(target_os = "windows", target_arch = "x86_64")
))]
fn cuda_execution_provider() -> ExecutionProviderDispatch {
    ep::CUDAExecutionProvider::default()
        .build()
        .error_on_failure()
}

fn configure_tensorrt_strict(
    session_builder: SessionBuilder,
    engine_cache_path: &str,
) -> RbtResult<SessionBuilder> {
    Ok(session_builder
        .with_execution_providers([tensorrt_execution_provider(engine_cache_path)])?)
}

#[cfg(any(
    all(
        target_os = "linux",
        any(target_arch = "aarch64", target_arch = "x86_64")
    ),
    all(target_os = "windows", target_arch = "x86_64")
))]
fn configure_cuda_strict(session_builder: SessionBuilder) -> RbtResult<SessionBuilder> {
    Ok(session_builder.with_execution_providers([cuda_execution_provider()])?)
}

#[cfg(not(any(
    all(
        target_os = "linux",
        any(target_arch = "aarch64", target_arch = "x86_64")
    ),
    all(target_os = "windows", target_arch = "x86_64")
)))]
fn configure_cuda_strict(_session_builder: SessionBuilder) -> RbtResult<SessionBuilder> {
    Err(RbtError::UnsupportedExecutionProvider(
        "CUDA is only enabled for aarch64/x86_64 Linux and x86_64 Windows targets".to_string(),
    ))
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn configure_auto_session_builder(
    session_builder: SessionBuilder,
    engine_cache_path: &str,
) -> RbtResult<(SessionBuilder, OrtExecutionProvider)> {
    let mut session_builder = session_builder;
    if linux_aarch64_tensorrt_stack_available() {
        match try_configure_execution_provider(
            session_builder,
            tensorrt_execution_provider(engine_cache_path),
        ) {
            Ok(session_builder) => return Ok((session_builder, OrtExecutionProvider::TensorRT)),
            Err(recovered) => {
                session_builder = recovered;
            }
        }
    }

    if linux_aarch64_cuda_stack_available() {
        match try_configure_execution_provider(session_builder, cuda_execution_provider()) {
            Ok(session_builder) => return Ok((session_builder, OrtExecutionProvider::CUDA)),
            Err(recovered) => {
                session_builder = recovered;
            }
        }
    }

    Ok((configure_cpu(session_builder)?, OrtExecutionProvider::CPU))
}

#[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
fn configure_auto_session_builder(
    session_builder: SessionBuilder,
    engine_cache_path: &str,
) -> RbtResult<(SessionBuilder, OrtExecutionProvider)> {
    let provider = default_execution_provider();
    let session_builder = match provider {
        OrtExecutionProvider::CoreML => configure_coreml(session_builder, engine_cache_path)?,
        OrtExecutionProvider::OpenVINO => configure_openvino(session_builder)?,
        OrtExecutionProvider::TensorRT => {
            configure_tensorrt_strict(session_builder, engine_cache_path)?
        }
        OrtExecutionProvider::CUDA => configure_cuda_strict(session_builder)?,
        OrtExecutionProvider::CPU => configure_cpu(session_builder)?,
    };
    Ok((session_builder, provider))
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn linux_aarch64_cuda_available() -> bool {
    const CUDA_DIRS: &[&str] = &[
        "/usr/local/cuda",
        "/usr/local/cuda/targets/aarch64-linux",
        "/usr/lib/aarch64-linux-gnu/tegra",
    ];
    const CUDA_LIB_DIRS: &[&str] = &[
        "/usr/lib/aarch64-linux-gnu",
        "/usr/lib/aarch64-linux-gnu/tegra",
        "/usr/local/cuda/lib64",
        "/usr/local/cuda/targets/aarch64-linux/lib",
    ];

    CUDA_DIRS
        .iter()
        .any(|path| std::path::Path::new(path).exists())
        || CUDA_LIB_DIRS
            .iter()
            .any(|dir| directory_has_any_library(dir, &["libcuda.so", "libcudart.so"]))
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn linux_aarch64_cudnn_available() -> bool {
    const CUDNN_LIB_DIRS: &[&str] = &[
        "/usr/lib/aarch64-linux-gnu",
        "/usr/lib/aarch64-linux-gnu/tegra",
        "/usr/local/cuda/lib64",
        "/usr/local/cuda/targets/aarch64-linux/lib",
    ];

    CUDNN_LIB_DIRS
        .iter()
        .any(|dir| directory_has_any_library(dir, &["libcudnn.so", "libcudnn.so.9"]))
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn linux_aarch64_tensorrt_available() -> bool {
    const TENSORRT_LIB_DIRS: &[&str] = &[
        "/usr/lib/aarch64-linux-gnu",
        "/usr/lib/aarch64-linux-gnu/tegra",
        "/usr/local/tensorrt/lib",
        "/usr/local/TensorRT/lib",
        "/opt/tensorrt/lib",
    ];

    TENSORRT_LIB_DIRS.iter().any(|dir| {
        directory_has_any_library(dir, &["libnvinfer.so", "libnvinfer.so.10"])
            && directory_has_any_library(dir, &["libnvinfer_plugin.so", "libnvinfer_plugin.so.10"])
    })
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn linux_aarch64_tensorrt_stack_available() -> bool {
    linux_aarch64_cuda_stack_available() && linux_aarch64_tensorrt_available()
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn linux_aarch64_cuda_stack_available() -> bool {
    linux_aarch64_cuda_available() && linux_aarch64_cudnn_available()
}

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn directory_has_any_library(dir: &str, library_prefixes: &[&str]) -> bool {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return false;
    };
    entries.filter_map(Result::ok).any(|entry| {
        let name = entry.file_name();
        let Some(name) = name.to_str() else {
            return false;
        };
        library_prefixes
            .iter()
            .any(|prefix| name == *prefix || name.starts_with(&format!("{prefix}.")))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_uses_target_default() {
        assert_eq!(
            resolve_execution_provider("auto"),
            Some(default_execution_provider())
        );
        assert_eq!(
            resolve_execution_provider(""),
            Some(default_execution_provider())
        );
    }

    #[test]
    fn explicit_provider_is_case_insensitive() {
        assert_eq!(
            resolve_execution_provider("coreml"),
            Some(OrtExecutionProvider::CoreML)
        );
        assert_eq!(
            resolve_execution_provider("OPENVINO"),
            Some(OrtExecutionProvider::OpenVINO)
        );
        assert_eq!(
            resolve_execution_provider("TensorRT"),
            Some(OrtExecutionProvider::TensorRT)
        );
        assert_eq!(
            resolve_execution_provider("cuda"),
            Some(OrtExecutionProvider::CUDA)
        );
        assert_eq!(
            resolve_execution_provider("cpu"),
            Some(OrtExecutionProvider::CPU)
        );
    }

    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    #[test]
    fn linux_aarch64_auto_uses_accelerator_or_cpu() {
        let provider = default_execution_provider();
        assert!(matches!(
            provider,
            OrtExecutionProvider::TensorRT | OrtExecutionProvider::CUDA | OrtExecutionProvider::CPU
        ));
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn macos_default_is_coreml() {
        assert_eq!(default_execution_provider(), OrtExecutionProvider::CoreML);
    }
}

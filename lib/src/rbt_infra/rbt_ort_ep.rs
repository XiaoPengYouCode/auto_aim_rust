use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use ort::{ep, session::builder::SessionBuilder};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum OrtExecutionProvider {
    CoreML,
    OpenVINO,
    TensorRT,
    CPU,
}

impl OrtExecutionProvider {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::CoreML => "CoreML",
            Self::OpenVINO => "OpenVINO",
            Self::TensorRT => "TensorRT",
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
        "cpu" => Some(OrtExecutionProvider::CPU),
        _ => None,
    }
}

pub fn configure_session_builder(
    session_builder: SessionBuilder,
    configured_ep: &str,
    engine_cache_path: &str,
) -> RbtResult<(SessionBuilder, OrtExecutionProvider)> {
    let provider = resolve_execution_provider(configured_ep)
        .ok_or_else(|| RbtError::UnsupportedExecutionProvider(configured_ep.to_string()))?;

    let session_builder = match provider {
        OrtExecutionProvider::CoreML => configure_coreml(session_builder, engine_cache_path)?,
        OrtExecutionProvider::OpenVINO => configure_openvino(session_builder)?,
        OrtExecutionProvider::TensorRT => {
            session_builder.with_execution_providers([ep::TensorRTExecutionProvider::default()
                .with_engine_cache(true)
                .with_engine_cache_path(engine_cache_path)
                .with_fp16(true)
                .build()
                .error_on_failure()])?
        }
        OrtExecutionProvider::CPU => {
            session_builder.with_execution_providers([ep::CPUExecutionProvider::default()
                .with_arena_allocator(true)
                .build()])?
        }
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
            resolve_execution_provider("cpu"),
            Some(OrtExecutionProvider::CPU)
        );
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn macos_default_is_coreml() {
        assert_eq!(default_execution_provider(), OrtExecutionProvider::CoreML);
    }
}

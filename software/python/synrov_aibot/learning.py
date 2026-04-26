"""Dataset collection, model training and inference helpers."""

from ._legacy import (
    SampleRecord,
    DatasetCollector,
    load_dataset,
    train_model,
    build_inference_feature,
    prepare_training_rows,
    vector_l2,
    cosine_similarity,
    clamp_infer_period_ms,
    compute_feature_familiarity,
    compute_tree_agreement,
    estimate_prediction_confidence,
    spontaneity_offsets,
    should_enable_spontaneity,
    blend_predictions,
    observed_shadow_vector,
    evaluate_shadow_score,
    load_count,
)
try:
    from ._legacy import BufferedDatasetCollector  # type: ignore
except Exception:  # pragma: no cover
    BufferedDatasetCollector = None  # type: ignore

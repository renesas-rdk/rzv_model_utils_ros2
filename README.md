# RZV Model Utils ROS2

RZV Model Utils ROS2 is a utility library that provides a collection of helper functions for integrating AI models into ROS 2 applications.

## Overview

This package acts as a bridge between ROS 2 nodes and the underlying C++ AI model implementations, following ROS 2 design principles for parameters, package lookup, and resource paths.

## Features
- Encoding detection or pose information into ROS 2 message types
- Generating diagnostic messages for inference timing
- Loading model configuration and metadata from ROS 2 packages

## Architecture

The package follows a layered architecture:

-   **ROS 2 Layer**:
    Your application nodes use this package to load model configurations and metadata from ROS 2 packages, prepare model inputs, and convert inference outputs into ROS 2 message types.

-   **Utility Layer (rzv_model_utils_ros2)**:

    Provides helper functions for model loading, bounding-box and keypoint encoding, and generating diagnostic messages.

-   **AI Backend Layer (rzv_model)**:
    Implements the core C++ inference engine, including BaseModel, YOLO model variants, and all related pre-processing and post-processing logic.

## Usage

### Model Loading
To register a new AI model, you must add an entry to the package’s YAML configuration.
Each entry defines the model path, class names file, and optional metadata used by load_model_info().

1. Add a model entry in YAML

    Example file:  `config/models/models_config.yaml`

    ```yaml
    models:
        model_custom:
            path: "models/model_custom"
            names:    # Class labels
            0: class_name0
            1: class_name1
            2: class_name2
    ```

2. Use the custom model in ROS Node

    ```cpp
    // Load model configuration from YAML
    // Fallback priority: User overrides → YAML config → package defaults
    std::string model_type_ = "model_custom";  // Name of the YAML entry

    auto object_model = rzv_model::UtilsROS::load_model_info(
        "current_package",          // ROS 2 package containing the model
        model_type_,                // Model type key (e.g., "model_custom")
        model_path_param,           // Optional path override
        class_names_param           // Optional class name override
    );

    // Extract resolved configuration
    model_path_ = object_model.model_path;
    class_names_ = object_model.class_names;

    ```

## Dependencies

- ROS2
- OpenCV
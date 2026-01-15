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

**1**. Place the YAML Configuration File

By default, the model configuration file must be located at:

```bash
<path-to-packages>/config/models/models_config.yaml
```

If your project is already set up, this file should already exist.

If not, you can create it manually:

- Create a config/ directory at the project root.
- Inside it, create a models/ directory.
- Add the `models_config.yaml` file into that folder. See the [Model Configuration Reference Template](config/models/models_config.yaml) for details.


If your project does not follow this structure, create it as shown below:

```bash
    <ROS2 packages>
    ├── CMakeLists.txt
    ├── config
    │   └── models
    │       ├── ai_model_name  ##  contains the output from RUHMI compilation.
    │       └── models_config.yaml
    └── ...
```

**2**. Use the custom model in ROS Node

```cpp
// Load model configuration from YAML
// Fallback priority: User overrides → YAML config → package defaults
std::string model_type_ = "ai_model_name";  // Name of the YAML entry

auto object_model = rzv_model::UtilsROS::load_model_info(
    "<ROS2 packages>",          // ROS 2 package containing the model
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
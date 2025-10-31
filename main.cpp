// Copyright 2025
// SPDX-License-Identifier: Apache-2.0

// AGX to USD Converter - Converts animated geometry from AGX format to USD

// AGX
#define AGX_READ_IMPL
#include "agx/agx_read.h"

// USD
#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/tf/token.h>

// std
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cstring>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

// Helper to convert AGX parameter name to a valid USD attribute name
std::string makeValidAttrName(const std::string &name)
{
  std::string result = name;
  // Replace '.' with '_' for USD attribute names
  for (char &c : result) {
    if (c == '.')
      c = '_';
  }
  return result;
}

// Helper to extract parameter name as std::string
std::string getParamName(const AGXParamView &pv)
{
  return std::string(pv.name, pv.nameLength);
}

// Structure to hold mesh data for a single timestep
struct MeshData
{
  std::vector<GfVec3f> vertices;
  std::vector<int> faceVertexCounts;
  std::vector<int> faceVertexIndices;
  std::vector<GfVec3f> normals;
  std::vector<GfVec2f> uvs;
  bool hasNormals = false;
  bool hasUVs = false;
};

// Convert AGX mesh data to USD mesh
bool convertToUSDMesh(AGXReader reader, const std::string &outputPath)
{
  // Read header
  AGXHeader hdr{};
  if (agxReaderGetHeader(reader, &hdr) != 0) {
    std::cerr << "Error: Failed to read AGX header\n";
    return false;
  }

  std::cout << "AGX File Info:\n";
  std::cout << "  Version: " << hdr.version << "\n";
  std::cout << "  Time Steps: " << hdr.timeSteps << "\n";
  std::cout << "  Constants: " << hdr.constantParamCount << "\n";
  std::cout << "  Object Type: " << anari::toString(hdr.objectType) << "\n";

  const char *subtype = agxReaderGetSubtype(reader);
  if (subtype && strlen(subtype) > 0) {
    std::cout << "  Subtype: " << subtype << "\n";
  }

  // Create USD stage (binary format with .usdc extension)
  auto stage = UsdStage::CreateNew(outputPath);
  if (!stage) {
    std::cerr << "Error: Failed to create USD stage\n";
    return false;
  }

  // Set standard USD metadata
  UsdGeomSetStageUpAxis(stage, TfToken("Y"));       // Y-up coordinate system
  UsdGeomSetStageMetersPerUnit(stage, 1.0);          // 1 unit = 1 meter
  
  // Set up time code settings
  double startTime = 0.0;
  double endTime = static_cast<double>(hdr.timeSteps > 0 ? hdr.timeSteps - 1 : 0);
  stage->SetStartTimeCode(startTime);
  stage->SetEndTimeCode(endTime);
  stage->SetTimeCodesPerSecond(24.0); // Standard framerate
  stage->SetFramesPerSecond(24.0);

  // Create root transform
  auto xform = UsdGeomXform::Define(stage, SdfPath("/Geometry"));
  
  // Set as default prim for the stage
  stage->SetDefaultPrim(xform.GetPrim());

  // Create mesh
  auto mesh = UsdGeomMesh::Define(stage, SdfPath("/Geometry/mesh"));

  // Store constant parameters
  std::map<std::string, std::vector<uint8_t>> constants;

  // Read constant parameters
  std::cout << "\nReading constant parameters...\n";
  agxReaderResetConstants(reader);
  AGXParamView pv{};
  
  std::vector<int> constantIndices;
  
  while (true) {
    int rc = agxReaderNextConstant(reader, &pv);
    if (rc < 0) {
      std::cerr << "Error reading constant parameters\n";
      return false;
    }
    if (rc == 0)
      break;

    std::string paramName = getParamName(pv);
    std::cout << "  " << paramName;
    
    if (!pv.isArray) {
      std::cout << " (scalar, type=" << anari::toString(pv.type) << ")\n";
    } else {
      std::cout << " (array, type=" << anari::toString(pv.elementType) 
                << ", count=" << pv.elementCount << ")\n";
      
      // Store array data for later use
      std::vector<uint8_t> data(pv.dataBytes);
      std::memcpy(data.data(), pv.data, pv.dataBytes);
      constants[paramName] = std::move(data);

      // Handle indices specially (topology is often constant)
      if (paramName == "primitive.index" || paramName == "index" || 
          paramName == "primitive.indices" || paramName == "indices") {
        
        if (pv.elementType == ANARI_UINT32_VEC3 || pv.elementType == ANARI_UINT32) {
          const uint32_t *indexData = reinterpret_cast<const uint32_t *>(pv.data);
          size_t numIndices = pv.dataBytes / sizeof(uint32_t);
          
          VtArray<int> indices(numIndices);
          for (size_t i = 0; i < numIndices; ++i) {
            indices[i] = static_cast<int>(indexData[i]);
          }
          
          mesh.GetFaceVertexIndicesAttr().Set(indices);
          
          // If these are triangle indices, set face vertex counts
          if (pv.elementType == ANARI_UINT32_VEC3 || (numIndices % 3 == 0)) {
            size_t numFaces = numIndices / 3;
            VtArray<int> faceCounts(numFaces, 3);
            mesh.GetFaceVertexCountsAttr().Set(faceCounts);
            std::cout << "    -> Set as mesh topology (" << numFaces << " triangles)\n";
          }
        }
      }
    }
  }

  // Process time steps
  std::cout << "\nProcessing time steps...\n";
  agxReaderResetTimeSteps(reader);
  
  uint32_t stepIndex = 0;
  uint32_t paramCount = 0;
  
  while (agxReaderBeginNextTimeStep(reader, &stepIndex, &paramCount) == 1) {
    std::cout << "Time step " << stepIndex << " (" << paramCount << " parameters)\n";
    double timeCode = static_cast<double>(stepIndex);
    
    // Read parameters for this timestep
    while (true) {
      int rc = agxReaderNextTimeStepParam(reader, &pv);
      if (rc < 0) {
        std::cerr << "Error reading timestep parameters\n";
        return false;
      }
      if (rc == 0)
        break;

      std::string paramName = getParamName(pv);
      
      // Handle vertex positions
      if (paramName == "vertex.position" || paramName == "position" || 
          paramName == "vertex.positions" || paramName == "positions") {
        
        if (pv.isArray && pv.elementType == ANARI_FLOAT32_VEC3) {
          const float *posData = reinterpret_cast<const float *>(pv.data);
          size_t numVerts = pv.elementCount;
          
          VtArray<GfVec3f> points(numVerts);
          for (size_t i = 0; i < numVerts; ++i) {
            points[i] = GfVec3f(posData[i * 3 + 0], 
                                posData[i * 3 + 1], 
                                posData[i * 3 + 2]);
          }
          
          mesh.GetPointsAttr().Set(points, timeCode);
          std::cout << "  -> Set " << numVerts << " vertex positions at time " << timeCode << "\n";
        }
      }
      // Handle normals
      else if (paramName == "vertex.normal" || paramName == "normal" || 
               paramName == "vertex.normals" || paramName == "normals") {
        
        if (pv.isArray && pv.elementType == ANARI_FLOAT32_VEC3) {
          const float *normData = reinterpret_cast<const float *>(pv.data);
          size_t numNormals = pv.elementCount;
          
          VtArray<GfVec3f> normals(numNormals);
          for (size_t i = 0; i < numNormals; ++i) {
            normals[i] = GfVec3f(normData[i * 3 + 0], 
                                 normData[i * 3 + 1], 
                                 normData[i * 3 + 2]);
          }
          
          auto normalsAttr = mesh.GetNormalsAttr();
          normalsAttr.Set(normals, timeCode);
          mesh.SetNormalsInterpolation(UsdGeomTokens->vertex);
          std::cout << "  -> Set " << numNormals << " normals at time " << timeCode << "\n";
        }
      }
      // Handle vertex.attribute0 as primvar (for shading/coloring)
      else if (paramName == "vertex.attribute0" || paramName == "attribute0") {
        
        if (pv.isArray) {
          UsdGeomPrimvarsAPI primvarsAPI(mesh);
          
          // Handle different attribute types
          if (pv.elementType == ANARI_FLOAT32) {
            // Scalar attribute (e.g., for color mapping)
            const float *data = reinterpret_cast<const float *>(pv.data);
            VtArray<float> values(pv.elementCount);
            for (size_t i = 0; i < pv.elementCount; ++i) {
              values[i] = data[i];
            }
            
            auto primvar = primvarsAPI.CreatePrimvar(TfToken("attribute0"), 
                                             SdfValueTypeNames->FloatArray,
                                             UsdGeomTokens->vertex);
            primvar.Set(values, timeCode);
            std::cout << "  -> Set scalar attribute0 (" << pv.elementCount << " values) at time " << timeCode << "\n";
          }
          else if (pv.elementType == ANARI_FLOAT32_VEC2) {
            // Vec2 attribute (e.g., UVs)
            const float *data = reinterpret_cast<const float *>(pv.data);
            VtArray<GfVec2f> values(pv.elementCount);
            for (size_t i = 0; i < pv.elementCount; ++i) {
              values[i] = GfVec2f(data[i * 2 + 0], data[i * 2 + 1]);
            }
            
            auto primvar = primvarsAPI.CreatePrimvar(TfToken("attribute0"), 
                                             SdfValueTypeNames->Float2Array,
                                             UsdGeomTokens->vertex);
            primvar.Set(values, timeCode);
            std::cout << "  -> Set vec2 attribute0 (" << pv.elementCount << " values) at time " << timeCode << "\n";
          }
          else if (pv.elementType == ANARI_FLOAT32_VEC3) {
            // Vec3 attribute (e.g., colors)
            const float *data = reinterpret_cast<const float *>(pv.data);
            VtArray<GfVec3f> values(pv.elementCount);
            for (size_t i = 0; i < pv.elementCount; ++i) {
              values[i] = GfVec3f(data[i * 3 + 0], data[i * 3 + 1], data[i * 3 + 2]);
            }
            
            auto primvar = primvarsAPI.CreatePrimvar(TfToken("attribute0"), 
                                             SdfValueTypeNames->Float3Array,
                                             UsdGeomTokens->vertex);
            primvar.Set(values, timeCode);
            std::cout << "  -> Set vec3 attribute0 (" << pv.elementCount << " values) at time " << timeCode << "\n";
          }
          else if (pv.elementType == ANARI_FLOAT32_VEC4) {
            // Vec4 attribute (e.g., RGBA colors)
            const float *data = reinterpret_cast<const float *>(pv.data);
            VtArray<GfVec4f> values(pv.elementCount);
            for (size_t i = 0; i < pv.elementCount; ++i) {
              values[i] = GfVec4f(data[i * 4 + 0], data[i * 4 + 1], data[i * 4 + 2], data[i * 4 + 3]);
            }
            
            auto primvar = primvarsAPI.CreatePrimvar(TfToken("attribute0"), 
                                             SdfValueTypeNames->Float4Array,
                                             UsdGeomTokens->vertex);
            primvar.Set(values, timeCode);
            std::cout << "  -> Set vec4 attribute0 (" << pv.elementCount << " values) at time " << timeCode << "\n";
          }
        }
      }
      // Handle UVs (separate from attribute0)
      else if (paramName == "uv" || paramName == "vertex.uv" || paramName == "texcoord") {
        
        if (pv.isArray && pv.elementType == ANARI_FLOAT32_VEC2) {
          const float *uvData = reinterpret_cast<const float *>(pv.data);
          size_t numUVs = pv.elementCount;
          
          VtArray<GfVec2f> uvs(numUVs);
          for (size_t i = 0; i < numUVs; ++i) {
            uvs[i] = GfVec2f(uvData[i * 2 + 0], uvData[i * 2 + 1]);
          }
          
          // Create primvar for UVs
          UsdGeomPrimvarsAPI primvarsAPI(mesh);
          auto primvar = primvarsAPI.CreatePrimvar(TfToken("st"), 
                                           SdfValueTypeNames->Float2Array,
                                           UsdGeomTokens->vertex);
          primvar.Set(uvs, timeCode);
          std::cout << "  -> Set " << numUVs << " UVs at time " << timeCode << "\n";
        }
      }
      // Handle triangle indices (topology can change per timestep)
      else if (paramName == "primitive.index" || paramName == "index" || 
               paramName == "primitive.indices" || paramName == "indices") {
        
        if (pv.isArray && pv.elementType == ANARI_UINT32_VEC3) {
          const uint32_t *indexData = reinterpret_cast<const uint32_t *>(pv.data);
          size_t numIndices = pv.elementCount * 3; // VEC3 = 3 indices per triangle
          
          VtArray<int> indices(numIndices);
          for (size_t i = 0; i < numIndices; ++i) {
            indices[i] = static_cast<int>(indexData[i]);
          }
          
          mesh.GetFaceVertexIndicesAttr().Set(indices, timeCode);
          
          // Set face vertex counts (all triangles = 3 vertices each)
          size_t numFaces = pv.elementCount;
          VtArray<int> faceCounts(numFaces, 3);
          mesh.GetFaceVertexCountsAttr().Set(faceCounts, timeCode);
          
          std::cout << "  -> Set mesh topology (" << numFaces << " triangles) at time " << timeCode << "\n";
        }
      }
      // Handle generic time parameter
      else if (paramName == "time") {
        if (!pv.isArray && pv.elementType == ANARI_UNKNOWN) {
          // Single value - might be useful for custom attributes
          std::cout << "  -> Time value parameter\n";
        }
      }
      // Handle other arrays as custom primvars
      else if (pv.isArray) {
        std::cout << "  -> Custom array: " << paramName 
                  << " (type=" << anari::toString(pv.elementType) 
                  << ", count=" << pv.elementCount << ")\n";
        
        // Could add custom primvars here for other attributes
      }
    }
  }

  // Save the stage
  std::cout << "\nSaving USD file to: " << outputPath << "\n";
  stage->GetRootLayer()->Save();
  
  std::cout << "Conversion complete!\n";
  std::cout << "Time range: " << startTime << " to " << endTime << "\n";
  
  return true;
}

} // anonymous namespace

int main(int argc, char **argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input.agx> <output.usdc>\n";
    std::cerr << "\n";
    std::cerr << "Converts AGX animated geometry files to USD binary format.\n";
    std::cerr << "The output file should have a .usdc extension for binary format.\n";
    return 1;
  }

  const char *inputPath = argv[1];
  const char *outputPath = argv[2];

  std::cout << "AGX to USD Converter\n";
  std::cout << "====================\n";
  std::cout << "Input:  " << inputPath << "\n";
  std::cout << "Output: " << outputPath << "\n\n";

  // Open AGX file
  AGXReader reader = agxNewReader(inputPath);
  if (!reader) {
    std::cerr << "Error: Failed to open AGX file: " << inputPath << "\n";
    return 2;
  }

  // Convert to USD
  bool success = convertToUSDMesh(reader, outputPath);

  // Cleanup
  agxReleaseReader(reader);

  return success ? 0 : 3;
}


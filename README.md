# hackathon
#code1
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.ARFoundation;


public class aaa : MonoBehaviour
{
    private ARTrackedImageManager _artTrachedImageManager;

    private void Awake()
    {
        _artTrachedImageManager = FindObjectOfType<ARTrackedImageManager>();

    }
    private void onEnable()
    {
        _artTrachedImageManager.trackedImagesChanged += onImageChanged;

    }
    private void onDnable()
    {
        _artTrachedImageManager.trackedImagesChanged -= onImageChanged;


    }
    private void onImageChanged(ARTrackedImagesChangedEventArgs args)
    {
        foreach (var trackedImage in args.added)
        {
            Debug.Log(trackedImage.name);
        }

    }



}
#code2
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;

//
// This script allows us to create anchors with
// a prefab attached in order to visbly discern where the anchors are created.
// Anchors are a particular point in space that you are asking your device to track.
//

[RequireComponent(typeof(ARAnchorManager))]
[RequireComponent(typeof(ARRaycastManager))]
[RequireComponent(typeof(ARPlaneManager))]
public class AnchorCreator : MonoBehaviour
{
    // This is the prefab that will appear every time an anchor is created.
    [SerializeField]
    GameObject m_AnchorPrefab;

    public GameObject AnchorPrefab
    {
        get => m_AnchorPrefab;
        set => m_AnchorPrefab = value;
    }

    // Removes all the anchors that have been created.
    public void RemoveAllAnchors()
    {
        foreach (var anchor in m_AnchorPoints)
        {
            Destroy(anchor);
        }
        m_AnchorPoints.Clear();
    }

    // On Awake(), we obtains a reference to all the required components.
    // The ARRaycastManager allows us to perform raycasts so that we know where to place an anchor.
    // The ARPlaneManager detects surfaces we can place our objects on.
    // The ARAnchorManager handles the processing of all anchors and updates their position and rotation.
    void Awake()
    {
        m_RaycastManager = GetComponent<ARRaycastManager>();
        m_AnchorManager = GetComponent<ARAnchorManager>();
        m_PlaneManager = GetComponent<ARPlaneManager>();
        m_AnchorPoints = new List<ARAnchor>();
    }

    void Update()
    {
        // If there is no tap, then simply do nothing until the next call to Update().
        if (Input.touchCount == 0)
            return;

        var touch = Input.GetTouch(0);
        if (touch.phase != TouchPhase.Began)
            return;

        if (m_RaycastManager.Raycast(touch.position, s_Hits, TrackableType.PlaneWithinPolygon))
        {
            // Raycast hits are sorted by distance, so the first one
            // will be the closest hit.
            var hitPose = s_Hits[0].pose;
            var hitTrackableId = s_Hits[0].trackableId;
            var hitPlane = m_PlaneManager.GetPlane(hitTrackableId);

            // This attaches an anchor to the area on the plane corresponding to the raycast hit,
            // and afterwards instantiates an instance of your chosen prefab at that point.
            // This prefab instance is parented to the anchor to make sure the position of the prefab is consistent
            // with the anchor, since an anchor attached to an ARPlane will be updated automatically by the ARAnchorManager as the ARPlane's exact position is refined.
            var anchor = m_AnchorManager.AttachAnchor(hitPlane, hitPose);

            Instantiate(m_AnchorPrefab, anchor.transform);

            if (anchor == null)
            {
                Debug.Log("Error creating anchor.");
            }
            else
            {
                // Stores the anchor so that it may be removed later.
                m_AnchorPoints.Add(anchor);
            }
        }
    }

    static List<ARRaycastHit> s_Hits = new List<ARRaycastHit>();

    List<ARAnchor> m_AnchorPoints;

    ARRaycastManager m_RaycastManager;

    ARAnchorManager m_AnchorManager;

    ARPlaneManager m_PlaneManager;
}

#code3
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;

/// <summary>
/// This plane visualizer demonstrates the use of a feathering effect
/// at the edge of the detected plane, which reduces the visual impression
/// of a hard edge.
/// </summary>
[RequireComponent(typeof(ARPlaneMeshVisualizer), typeof(MeshRenderer), typeof(ARPlane))]
public class ARFeatheredPlaneMeshVisualizer : MonoBehaviour
{
    [Tooltip("The width of the texture feathering (in world units).")]
    [SerializeField]
    float m_FeatheringWidth = 0.2f;

    /// <summary>
    /// The width of the texture feathering (in world units).
    /// </summary>
    public float featheringWidth
    {
        get { return m_FeatheringWidth; }
        set { m_FeatheringWidth = value; }
    }

    void Awake()
    {
        m_PlaneMeshVisualizer = GetComponent<ARPlaneMeshVisualizer>();
        m_FeatheredPlaneMaterial = GetComponent<MeshRenderer>().material;
        m_Plane = GetComponent<ARPlane>();
    }

    void OnEnable()
    {
        m_Plane.boundaryChanged += ARPlane_boundaryUpdated;
    }

    void OnDisable()
    {
        m_Plane.boundaryChanged -= ARPlane_boundaryUpdated;
    }

    void ARPlane_boundaryUpdated(ARPlaneBoundaryChangedEventArgs eventArgs)
    {
        GenerateBoundaryUVs(m_PlaneMeshVisualizer.mesh);
    }

    /// <summary>
    /// Generate UV2s to mark the boundary vertices and feathering UV coords.
    /// </summary>
    /// <remarks>
    /// The <c>ARPlaneMeshVisualizer</c> has a <c>meshUpdated</c> event that can be used to modify the generated
    /// mesh. In this case we'll add UV2s to mark the boundary vertices.
    /// This technique avoids having to generate extra vertices for the boundary. It works best when the plane is
    /// is fairly uniform.
    /// </remarks>
    /// <param name="mesh">The <c>Mesh</c> generated by <c>ARPlaneMeshVisualizer</c></param>
    void GenerateBoundaryUVs(Mesh mesh)
    {
        int vertexCount = mesh.vertexCount;

        // Reuse the list of UVs
        s_FeatheringUVs.Clear();
        if (s_FeatheringUVs.Capacity < vertexCount) { s_FeatheringUVs.Capacity = vertexCount; }

        mesh.GetVertices(s_Vertices);

        Vector3 centerInPlaneSpace = s_Vertices[s_Vertices.Count - 1];
        Vector3 uv = new Vector3(0, 0, 0);
        float shortestUVMapping = float.MaxValue;

        // Assume the last vertex is the center vertex.
        for (int i = 0; i < vertexCount - 1; i++)
        {
            float vertexDist = Vector3.Distance(s_Vertices[i], centerInPlaneSpace);

            // Remap the UV so that a UV of "1" marks the feathering boudary.
            // The ratio of featherBoundaryDistance/edgeDistance is the same as featherUV/edgeUV.
            // Rearrange to get the edge UV.
            float uvMapping = vertexDist / Mathf.Max(vertexDist - featheringWidth, 0.001f);
            uv.x = uvMapping;

            // All the UV mappings will be different. In the shader we need to know the UV value we need to fade out by.
            // Choose the shortest UV to guarentee we fade out before the border.
            // This means the feathering widths will be slightly different, we again rely on a fairly uniform plane.
            if (shortestUVMapping > uvMapping) { shortestUVMapping = uvMapping; }

            s_FeatheringUVs.Add(uv);
        }

        m_FeatheredPlaneMaterial.SetFloat("_ShortestUVMapping", shortestUVMapping);

        // Add the center vertex UV
        uv.Set(0, 0, 0);
        s_FeatheringUVs.Add(uv);

        mesh.SetUVs(1, s_FeatheringUVs);
        mesh.UploadMeshData(false);
    }

    static List<Vector3> s_FeatheringUVs = new List<Vector3>();

    static List<Vector3> s_Vertices = new List<Vector3>();

    ARPlaneMeshVisualizer m_PlaneMeshVisualizer;

    ARPlane m_Plane;

    Material m_FeatheredPlaneMaterial;
}

#code4
using System;
using System.Collections.Generic;
using Unity.Jobs;
using UnityEngine.Serialization;
using UnityEngine.XR.ARSubsystems;

namespace UnityEngine.XR.ARFoundation
{
    /// <summary>
    /// A manager for <see cref="ARTrackedImage"/>s. Uses the <c>XRImageTrackingSubsystem</c>
    /// to recognize and track 2D images in the physical environment.
    /// </summary>
    [DefaultExecutionOrder(ARUpdateOrder.k_TrackedImageManager)]
    [RequireComponent(typeof(ARSessionOrigin))]
    [HelpURL(HelpUrls.ApiWithNamespace + nameof(ARTrackedImageManager) + ".html")]
    public sealed class ARTrackedImageManager : ARTrackableManager<
        XRImageTrackingSubsystem,
        XRImageTrackingSubsystemDescriptor,
        XRImageTrackingSubsystem.Provider,
        XRTrackedImage,
        ARTrackedImage>
    {
        [SerializeField]
        [FormerlySerializedAs("m_ReferenceLibrary")]
        [Tooltip("The library of images which will be detected and/or tracked in the physical environment.")]
        XRReferenceImageLibrary m_SerializedLibrary;

        /// <summary>
        /// Get or set the reference image library (that is, the set of images to search for in the physical environment).
        /// </summary>
        /// <remarks>
        /// An <c>IReferenceImageLibrary</c> can be either an <c>XRReferenceImageLibrary</c>
        /// or a <c>RuntimeReferenceImageLibrary</c>. <c>XRReferenceImageLibrary</c>s can only be
        /// constructed in the Editor and are immutable at runtime. A <c>RuntimeReferenceImageLibrary</c>
        /// is the runtime representation of a <c>XRReferenceImageLibrary</c> and can be mutable
        /// at runtime (see <c>MutableRuntimeReferenceImageLibrary</c>).
        /// </remarks>
        /// <exception cref="System.InvalidOperationException">Thrown if the <see cref="referenceLibrary"/> is set to <c>null</c> while image tracking is enabled.</exception>
        public IReferenceImageLibrary referenceLibrary
        {
            get
            {
                if (subsystem != null)
                {
                    return subsystem.imageLibrary;
                }
                else
                {
                    return m_SerializedLibrary;
                }
            }

            set
            {
                if (value == null && subsystem != null && subsystem.running)
                    throw new InvalidOperationException("Cannot set a null reference library while image tracking is enabled.");

                if (value is XRReferenceImageLibrary serializedLibrary)
                {
                    m_SerializedLibrary = serializedLibrary;
                    if (subsystem != null)
                        subsystem.imageLibrary = subsystem.CreateRuntimeLibrary(serializedLibrary);
                }
                else if (value is RuntimeReferenceImageLibrary runtimeLibrary)
                {
                    m_SerializedLibrary = null;
                    EnsureSubsystemInstanceSet();

                    if (subsystem != null)
                        subsystem.imageLibrary = runtimeLibrary;
                }

                if (subsystem != null)
                    UpdateReferenceImages(subsystem.imageLibrary);
            }
        }

        /// <summary>
        /// Creates a <c>UnityEngine.XR.ARSubsystems.RuntimeReferenceImageLibrary</c> from an existing
        /// <c>UnityEngine.XR.ARSubsystems.XRReferenceImageLibrary</c>
        /// or an empty library if <paramref name="serializedLibrary"/> is <c>null</c>.
        /// Use this to construct reference image libraries at runtime. If the library is of type
        /// <c>MutableRuntimeReferenceImageLibrary</c>, it is modifiable at runtime.
        /// </summary>
        /// <param name="serializedLibrary">An existing <c>XRReferenceImageLibrary</c>, or <c>null</c> to create an empty mutable image library.</param>
        /// <returns>A new <c>RuntimeReferenceImageLibrary</c> representing the deserialized version of <paramref name="serializedLibrary"/> or an empty library if <paramref name="serializedLibrary"/> is <c>null</c>.</returns>
        /// <exception cref="System.NotSupportedException">Thrown if there is no subsystem. This usually means image tracking is not supported.</exception>
        public RuntimeReferenceImageLibrary CreateRuntimeLibrary(XRReferenceImageLibrary serializedLibrary = null)
        {
            EnsureSubsystemInstanceSet();

            if (subsystem == null)
                throw new NotSupportedException("No image tracking subsystem found. This usually means image tracking is not supported.");

            return subsystem.CreateRuntimeLibrary(serializedLibrary);
        }

        [SerializeField]
        [Tooltip("The maximum number of moving images to track in realtime. Not all implementations support this feature.")]
        int m_MaxNumberOfMovingImages;

        /// <summary>
        /// The maximum number of moving images to track in real time.
        /// This property is obsolete.
        /// Use <see cref="requestedMaxNumberOfMovingImages"/>
        /// or  <see cref="currentMaxNumberOfMovingImages"/> instead.
        /// </summary>
        [Obsolete("Use requestedMaxNumberOfMovingImages or currentMaxNumberOfMovingImages instead. (2020-01-16)")]
        public int maxNumberOfMovingImages
        {
            get => m_MaxNumberOfMovingImages;
            set => requestedMaxNumberOfMovingImages = value;
        }

        bool supportsMovingImages => descriptor?.supportsMovingImages == true;

        /// <summary>
        /// The requested maximum number of moving images to track in real time. Support can vary between devices and providers. Check
        /// for support at runtime with <see cref="SubsystemLifecycleManager{TSubsystem,TSubsystemDescriptor,TProvider}.descriptor"/>'s
        /// `supportsMovingImages` property.
        /// </summary>
        public int requestedMaxNumberOfMovingImages
        {
            get => supportsMovingImages ? subsystem.requestedMaxNumberOfMovingImages : m_MaxNumberOfMovingImages;
            set
            {
                m_MaxNumberOfMovingImages = value;
                 if (enabled && (descriptor?.supportsMovingImages == true))
                {
                    subsystem.requestedMaxNumberOfMovingImages = value;
                }
            }
        }

        /// <summary>
        /// Get the maximum number of moving images to track in real time that is currently in use by the subsystem.
        /// </summary>
        public int currentMaxNumberOfMovingImages => supportsMovingImages ? subsystem.currentMaxNumberOfMovingImages : 0;

        [SerializeField]
        [Tooltip("If not null, instantiates this prefab for each detected image.")]
        GameObject m_TrackedImagePrefab;

        /// <summary>
        /// If not null, instantiates this Prefab for each detected image.
        /// </summary>
        /// <remarks>
        /// The purpose of this property is to extend the functionality of <see cref="ARTrackedImage"/>s.
        /// It is not the recommended way to instantiate content associated with an <see cref="ARTrackedImage"/>.
        /// See [Tracked Image Prefab](xref:arfoundation-tracked-image-manager#tracked-image-prefab) for more details.
        /// </remarks>
        public GameObject trackedImagePrefab
        {
            get => m_TrackedImagePrefab;
            set => m_TrackedImagePrefab = value;
        }

        /// <summary>
        /// Get the Prefab that will be instantiated for each <see cref="ARTrackedImage"/>.
        /// </summary>
        /// <returns>The Prefab that will be instantiated for each <see cref="ARTrackedImage"/>.</returns>
        protected override GameObject GetPrefab() => m_TrackedImagePrefab;

        /// <summary>
        /// Invoked once per frame with information about the <see cref="ARTrackedImage"/>s that have changed (that is, been added, updated, or removed).
        /// This happens just before <see cref="ARTrackedImage"/>s are destroyed, so you can set <c>ARTrackedImage.destroyOnRemoval</c> to <c>false</c>
        /// from this event to suppress this behavior.
        /// </summary>
        public event Action<ARTrackedImagesChangedEventArgs> trackedImagesChanged;

        /// <summary>
        /// The name to be used for the <c>GameObject</c> whenever a new image is detected.
        /// </summary>
        protected override string gameObjectName => nameof(ARTrackedImage);

        /// <summary>
        /// Sets the image library on the subsystem before Start() is called on the <c>XRImageTrackingSubsystem</c>.
        /// </summary>
        protected override void OnBeforeStart()
        {
            if (subsystem.imageLibrary == null && m_SerializedLibrary != null)
            {
                subsystem.imageLibrary = subsystem.CreateRuntimeLibrary(m_SerializedLibrary);
                m_SerializedLibrary = null;
            }

            UpdateReferenceImages(subsystem.imageLibrary);
            if (supportsMovingImages)
            {
                subsystem.requestedMaxNumberOfMovingImages = m_MaxNumberOfMovingImages;
            }

            enabled = (subsystem.imageLibrary != null);
#if DEVELOPMENT_BUILD
            if (subsystem.imageLibrary == null)
            {
                Debug.LogWarning($"{nameof(ARTrackedImageManager)} '{name}' was enabled but no reference image library is specified. To enable, set a valid reference image library and then re-enable this component.");
            }
#endif
        }

        bool FindReferenceImage(Guid guid, out XRReferenceImage referenceImage)
        {
            if (m_ReferenceImages.TryGetValue(guid, out referenceImage))
                return true;

            // If we are using a mutable library, then it's possible an image
            // has been added that we don't yet know about, so search the library.
            if (referenceLibrary is MutableRuntimeReferenceImageLibrary mutableLibrary)
            {
                foreach (var candidateImage in mutableLibrary)
                {
                    if (candidateImage.guid.Equals(guid))
                    {
                        referenceImage = candidateImage;
                        m_ReferenceImages.Add(referenceImage.guid, referenceImage);
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Invoked just after updating each <see cref="ARTrackedImage"/>. Used to update the <see cref="ARTrackedImage.referenceImage"/>.
        /// </summary>
        /// <param name="image">The tracked image being updated.</param>
        /// <param name="sessionRelativeData">New data associated with the tracked image. Spatial data is
        /// relative to the <see cref="ARSessionOrigin"/>.</param>
        protected override void OnAfterSetSessionRelativeData(
            ARTrackedImage image,
            XRTrackedImage sessionRelativeData)
        {
            if (FindReferenceImage(sessionRelativeData.sourceImageId, out XRReferenceImage referenceImage))
            {
                image.referenceImage = referenceImage;
            }
#if DEVELOPMENT_BUILD
            else
            {
                Debug.LogError($"Could not find reference image with guid {sessionRelativeData.sourceImageId}");
            }
#endif
        }

        /// <summary>
        /// Invokes the <see cref="trackedImagesChanged"/> event.
        /// </summary>
        /// <param name="added">A list of images added this frame.</param>
        /// <param name="updated">A list of images updated this frame.</param>
        /// <param name="removed">A list of images removed this frame.</param>
        protected override void OnTrackablesChanged(
            List<ARTrackedImage> added,
            List<ARTrackedImage> updated,
            List<ARTrackedImage> removed)
        {
            if (trackedImagesChanged != null)
            {
                using (new ScopedProfiler("OnTrackedImagesChanged"))
                trackedImagesChanged(
                    new ARTrackedImagesChangedEventArgs(
                        added,
                        updated,
                        removed));
            }
        }

        void UpdateReferenceImages(RuntimeReferenceImageLibrary library)
        {
            if (library == null)
                return;

            int count = library.count;
            for (int i = 0; i < count; ++i)
            {
                var referenceImage = library[i];
                m_ReferenceImages[referenceImage.guid] = referenceImage;
            }
        }

        Dictionary<Guid, XRReferenceImage> m_ReferenceImages = new Dictionary<Guid, XRReferenceImage>();
    }
}






using UnityEngine;

public class TestDir : MonoBehaviour
{
    [Header("References")]
    public DroneController droneController;  // Drag your DroneController here
    public Transform arrowObject;            // Drag an arrow/cone GameObject here
    public LineRenderer rayLine;             // LineRenderer component for ray

    [Header("Ray Settings")]
    public float rayLength = 5f;
    public LayerMask raycastMask = -1;       // What layers to hit

    void Start()
    {
        if (rayLine == null)
            rayLine = gameObject.AddComponent<LineRenderer>();

        SetupLineRenderer();
    }

    void Update()
    {
        if (droneController == null || arrowObject == null) return;

        // Read inputDir from DroneController (your normalized input)
        Vector3 inputDir = droneController.rawInputDir;

        if (inputDir == Vector3.zero)
        {
            HideVisualizer();
            return;
        }

        // Face arrow toward input direction
        arrowObject.rotation = Quaternion.LookRotation(inputDir);
        arrowObject.gameObject.SetActive(true);

        // Shoot ray in input direction
        ShowRay(inputDir);

        // Optional: Debug raycast hit
        if (Physics.Raycast(transform.position, inputDir, out RaycastHit hit, rayLength, raycastMask))
        {
            Debug.DrawLine(transform.position, hit.point, Color.red, 0.1f);
        }
    }

    void ShowRay(Vector3 direction)
    {
        rayLine.enabled = true;
        rayLine.SetPosition(0, transform.position);
        rayLine.SetPosition(1, transform.position + direction * rayLength);
    }

    void HideVisualizer()
    {
        arrowObject.gameObject.SetActive(false);
        rayLine.enabled = false;
    }

    void SetupLineRenderer()
    {
        rayLine.material = new Material(Shader.Find("Sprites/Default"));
        rayLine.colorGradient = new Gradient()
        {
            colorKeys = new GradientColorKey[] { new GradientColorKey(Color.blue, 0f), new GradientColorKey(Color.blue, 1f) }
        };
        rayLine.startWidth = 0.05f;
        rayLine.endWidth = 0.02f;
        rayLine.enabled = false;
    }
}

using UnityEngine;
using UnityEngine.UI;

public class ImagePopup : MonoBehaviour
{
    public Canvas canvas;   // Reference to the canvas object
    public Image image;     // Reference to the image component
    public Button closeButton;  // Reference to the close button component

    // Call this method to show the image
    public void ShowImage(Texture2D texture)
    {
        // Set the texture to the image component
        image.sprite = Sprite.Create(texture, new Rect(0, 0, texture.width, texture.height), Vector2.zero);

        // Enable the canvas
        canvas.enabled = true;
    }

    // Call this method to hide the image
    public void HideImage()
    {
        // Disable the canvas
        canvas.enabled = false;
    }

    void Start()
    {
        // Set the close button's click event to call the HideImage method
        closeButton.onClick.AddListener(HideImage);
    }
}

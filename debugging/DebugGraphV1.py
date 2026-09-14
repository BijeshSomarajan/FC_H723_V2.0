import pandas as pd
import numpy as np
import plotly.graph_objects as go
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

CSV_FILE = "Log6.csv"

# True:
#   All signals are normalized so they can be compared on
#   one graph despite having different units/scales.
#
# False:
#   Original/raw values are plotted.
NORMALIZE = False

# Number of samples shown initially.
# Use None to show the complete CSV.
INITIAL_VISIBLE_SAMPLES = None


# ============================================================
# VALUE PRESETS (OFFSETS)
# ============================================================
#
# Optional per-column offsets that get ADDED to a signal's
# values right before it is plotted. Useful for:
#
#   - visually separating overlapping signals
#     (e.g. shift one gyro axis up so it doesn't sit on top
#      of another)
#   - applying a quick calibration / bias correction
#
# Key   = exact column name as it appears in the CSV
# Value = number added to every sample of that column
#
# Any column not listed here gets an offset of 0 (unchanged).
#
# Set APPLY_PRESETS = False to disable this feature entirely
# without having to delete your preset dictionary.
# ------------------------------------------------------------

APPLY_PRESETS = True

VALUE_PRESETS = {
   "controlData.throttleControl":-300
}


# ============================================================
# LOAD CSV
# ============================================================

df = pd.read_csv(CSV_FILE)

# Remove accidental spaces around column names
df.columns = df.columns.str.strip()

print("\nCSV loaded:")
print(f"Rows    : {len(df)}")
print(f"Columns : {len(df.columns)}")

print("\nColumns:")
for i, column in enumerate(df.columns):
    print(f"{i:3d}: {column}")


# ============================================================
# SAMPLE AXIS
# ============================================================

x = np.arange(len(df))


# ============================================================
# NORMALIZATION
# ============================================================

def normalize_signal(series):
    """
    Normalize a signal to approximately -1 ... +1.

    NaN values remain NaN.
    Constant signals become zero.
    """

    s = pd.to_numeric(series, errors="coerce")

    minimum = s.min()
    maximum = s.max()

    if pd.isna(minimum) or pd.isna(maximum):
        return s

    if maximum == minimum:
        return pd.Series(
            0.0,
            index=s.index
        )

    return (
        2.0 *
        ((s - minimum) / (maximum - minimum))
        - 1.0
    )


# ============================================================
# CREATE FIGURE
# ============================================================

fig = go.Figure()

numeric_columns = []
non_numeric_columns = []
preset_columns = []

for column in df.columns:

    # Try converting the column to numeric
    raw = pd.to_numeric(
        df[column],
        errors="coerce"
    )

    # Determine whether this is actually numeric
    valid_count = raw.notna().sum()

    if valid_count == 0:

        non_numeric_columns.append(column)

        print(
            f"Skipping non-numeric column: {column}"
        )

        continue

    numeric_columns.append(column)

    # --------------------------------------------------------
    # Normalize or use raw data
    # --------------------------------------------------------

    if NORMALIZE:
        y = normalize_signal(raw)
    else:
        y = raw

    # --------------------------------------------------------
    # Apply preset offset (if any)
    # --------------------------------------------------------

    offset = 0

    if APPLY_PRESETS:

        offset = VALUE_PRESETS.get(column, 0)

        if offset != 0:

            y = y + offset

            preset_columns.append(
                (column, offset)
            )

            print(
                f"Applying preset offset "
                f"{offset:+g} to column: {column}"
            )

    # --------------------------------------------------------
    # Store raw value in customdata.
    #
    # This means the graph can display the original value
    # even when the visible graph is normalized and/or
    # shifted by a preset offset.
    # --------------------------------------------------------

    customdata = np.column_stack(
        (
            raw.to_numpy(),
        )
    )

    # --------------------------------------------------------
    # Legend name shows the offset, if any, so it's obvious
    # a trace has been shifted from its true value.
    # --------------------------------------------------------

    if offset != 0:
        display_name = f"{column} ({offset:+g})"
    else:
        display_name = column

    # --------------------------------------------------------
    # Hover text: always show the true raw value, and also
    # show the offset / plotted value when an offset is
    # applied so nothing is misread as the real value.
    # --------------------------------------------------------

    if offset != 0:
        hovertemplate = (
            "<b>%{fullData.name}</b><br>"
            "Sample: %{x}<br>"
            "Raw value: %{customdata[0]:.6g}<br>"
            f"Offset applied: {offset:+.6g}<br>"
            "Plotted value: %{y:.6g}"
            "<extra></extra>"
        )
    else:
        hovertemplate = (
            "<b>%{fullData.name}</b><br>"
            "Sample: %{x}<br>"
            "Raw value: %{customdata[0]:.6g}"
            "<extra></extra>"
        )

    # --------------------------------------------------------
    # Add trace
    # --------------------------------------------------------

    fig.add_trace(
        go.Scatter(
            x=x,
            y=y,

            mode="lines",

            name=display_name,

            # All traces start visible
            visible=True,

            customdata=customdata,

            hovertemplate=hovertemplate,

            line=dict(
                width=1.3
            ),
        )
    )


# ============================================================
# INITIAL RANGE
# ============================================================

if INITIAL_VISIBLE_SAMPLES is not None:

    initial_end = min(
        INITIAL_VISIBLE_SAMPLES,
        len(df)
    )

    initial_range = [
        0,
        initial_end - 1
    ]

else:

    initial_range = [
        0,
        len(df) - 1
    ]


# ============================================================
# SHOW / HIDE ALL BUTTONS
# ============================================================

number_of_traces = len(numeric_columns)

show_all_visibility = [
    True
] * number_of_traces

hide_all_visibility = [
    False
] * number_of_traces


# ============================================================
# LAYOUT
# ============================================================

if NORMALIZE:

    y_axis_title = (
        "Normalized value (-1 ... +1)"
    )

else:

    y_axis_title = "Raw value"

if preset_columns:

    y_axis_title += " (some signals offset — see legend)"


title_text = (
    f"CSV Flight Log — "
    f"{len(numeric_columns)} Signals"
)

if preset_columns:

    title_text += (
        f" — {len(preset_columns)} preset offset(s) applied"
    )


fig.update_layout(

    title=dict(
        text=title_text,
        x=0.5
    ),

    xaxis=dict(
        title="Sample",
        range=initial_range,

        showgrid=True,
        zeroline=True,

        rangeslider=dict(
            visible=True
        ),

        rangeselector=dict(
            buttons=[
                dict(
                    count=100,
                    label="100",
                    step="all",
                    stepmode="backward"
                ),

                dict(
                    count=250,
                    label="250",
                    step="all",
                    stepmode="backward"
                ),

                dict(
                    count=500,
                    label="500",
                    step="all",
                    stepmode="backward"
                ),

                dict(
                    step="all",
                    label="ALL"
                ),
            ]
        )
    ),

    yaxis=dict(
        title=y_axis_title,

        showgrid=True,
        zeroline=True,

        fixedrange=False,
    ),

    # This makes the vertical crosshair useful when
    # comparing many signals at the same sample.
    hovermode="x unified",

    height=850,

    template="plotly_white",

    # Put the large legend on the right.
    legend=dict(
        title=dict(
            text=(
                "CSV Columns<br>"
                "<sup>Click = toggle</sup>"
            )
        ),

        orientation="v",

        yanchor="top",
        y=1,

        xanchor="left",
        x=1.02,

        itemclick="toggle",

        itemdoubleclick="toggleothers",

        groupclick="toggleitem",
    ),

    # Buttons above the graph
    updatemenus=[
        dict(

            type="buttons",

            direction="left",

            buttons=[

                dict(
                    label="SHOW ALL",

                    method="update",

                    args=[
                        {
                            "visible":
                                show_all_visibility
                        }
                    ]
                ),

                dict(
                    label="HIDE ALL",

                    method="update",

                    args=[
                        {
                            "visible":
                                hide_all_visibility
                        }
                    ]
                ),
            ],

            x=0,
            y=1.16,

            xanchor="left",
            yanchor="top",

            showactive=False,
        )
    ],

    margin=dict(
        l=80,
        r=300,
        t=130,
        b=100
    )
)


# ============================================================
# SAVE HTML
# ============================================================

output_file = (
    Path(CSV_FILE).with_suffix(".html")
)

fig.write_html(
    output_file,
    include_plotlyjs=True
)


# ============================================================
# INFORMATION
# ============================================================

print("\n--------------------------------------------")
print("Interactive graph created")
print("--------------------------------------------")

print(
    f"Numeric signals : {len(numeric_columns)}"
)

if non_numeric_columns:

    print(
        f"Non-numeric columns skipped : "
        f"{len(non_numeric_columns)}"
    )

if preset_columns:

    print(
        f"Preset offsets applied : {len(preset_columns)}"
    )

    for column, offset in preset_columns:

        print(
            f"  • {column}: {offset:+g}"
        )

print(
    f"\nOutput: {output_file}"
)

print(
    "\nControls:"
    "\n  • Click legend item       = show/hide signal"
    "\n  • Double-click legend     = isolate signal"
    "\n  • SHOW ALL                = show everything"
    "\n  • HIDE ALL                = hide everything"
    "\n  • Mouse wheel             = zoom"
    "\n  • Drag                    = zoom region"
    "\n  • Range slider             = navigate through log"
    "\n  • Hover                   = show raw CSV value"
)

print("--------------------------------------------")


# ============================================================
# OPEN GRAPH
# ============================================================

fig.show()
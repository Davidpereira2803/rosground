export const theme = {
  // Background colors — wider value steps for clear layering
  background: {
    primary: '#08090F',      // Near-black, slightly blue-tinted
    secondary: '#0F1320',    // Clear step up from primary
    card: '#161C2E',         // Noticeably distinct card surface
    input: '#1D2540',        // Active input surface
  },

  // Text colors — stronger hierarchy
  text: {
    primary: '#CDD6E8',      // Slightly desaturated white — less harsh
    secondary: '#7B90B2',    // Blue-grey mid tone
    accent: '#7AAED4',       // Dusty steel blue — calm, not vibrant
    muted: '#445570',        // Low-key, clearly subordinate
    placeholder: '#2E3F58',  // Barely there
  },

  // Accent colors — desaturated, purposeful
  accent: {
    primary: '#4A7FA5',      // Steel blue — muted, professional
    secondary: '#3A5A7A',    // Darker steel for secondary actions
    success: '#4E9068',      // Muted sage green
    error: '#A84848',        // Muted brick red
    warning: '#9A7540',      // Muted amber
  },

  // Border colors — visible but not loud
  border: {
    primary: '#1F2E48',      // Clear but subtle
    secondary: '#2A3F60',    // Slightly brighter for active states
    subtle: '#111827',       // Near-invisible dividers
  },

  // Status
  status: {
    connected: '#4E9068',
    disconnected: '#A84848',
  },
};